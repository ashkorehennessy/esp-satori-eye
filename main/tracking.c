#include "tracking.h"
#include "PID.h"
#include "context.h"
#include "servo.h"
#include "ai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "Tracking";

// =============================================
// 运行时可调参数（默认值，可通过 Web UI 修改）
// =============================================
static float s_kp = 0.06f;
static float s_ki = 0.02f;
static float s_kd = 0.0f;
static float s_max_increment = 30.0f;

// 设定点：画面中心
#define SETPOINT_X  ((float)AI_W / 2.0f)   // 120.0
#define SETPOINT_Y  ((float)AI_H / 2.0f)   // 120.0

// 舵机行程限位（硬件实测值）
#define SERVO_X_MIN   65
#define SERVO_X_MAX   135
#define SERVO_X_MID   100
#define SERVO_Y_MIN   65
#define SERVO_Y_MAX   120
#define SERVO_Y_MID   90

static PID_Incremental pid_x;
static PID_Incremental pid_y;

// PID 增量限幅
static float prev_angle_x;
static float prev_angle_y;

// 限幅辅助
static inline float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// === 动态 PID 增益调度（高斯 U 型）===
// ratio = scale * (1 - exp(-(error/sigma)²)) + base
// 中心附近增益平坦低，远离中心平滑上升
#define KP_RATIO_SCALE  0.7f
#define KP_RATIO_BASE   0.3f
#define KP_RATIO_SENS   40.0f

#define KI_RATIO_SCALE  0.7f
#define KI_RATIO_BASE   0.3f
#define KI_RATIO_SIGMA  50.0f

// =============================================
// 死区控制：误差持续在死区内则停止更新舵机
// =============================================
#define DEADZONE_PX       30        // 误差死区（像素）
#define DEADZONE_TIME_US  300000    // 持续 300ms 进入死区

static int64_t deadzone_enter_time;     // 进入死区的时间戳
static bool in_deadzone;                // 当前是否在死区中

// =============================================
// 检测间插值（线性外推）
// =============================================
#define INTERP_COUNT  2     // 每两次检测之间插入的 PID 调用次数

static float prev_det_cx, prev_det_cy;       // 上一次检测中心
static float vel_px_cx, vel_px_cy;           // 速度（像素/微秒）
static int64_t last_det_time;                // 上次检测时间
static int64_t det_interval;                 // 两次检测的间隔（微秒）
static volatile int interp_step;             // 当前已完成的插值步数
static bool has_prev_det;                    // 是否有上一次检测数据

// =============================================
// 边界丢失追逐
// =============================================
#define EDGE_THRESHOLD  15          // 检测框边缘距画面边界 < 15px 视为靠近边界
#define EDGE_CHASE_US   2000000     // 边界追逐持续时间 2000ms

// 上一次检测框的边界
static int last_x1, last_y1, last_x2, last_y2;
static float last_cx, last_cy;

// 边界追逐状态
static volatile bool edge_chasing;
static float edge_chase_cx, edge_chase_cy;  // 追逐目标坐标
static int64_t edge_chase_start;            // 追逐开始时间

void tracking_init(void) {
    // 增量式 PID，输出范围为实际舵机行程
    pid_x = PID_Incremental_Init(s_kp, s_ki, s_kd, SERVO_X_MAX, SERVO_X_MIN, false, 0.5f);
    pid_y = PID_Incremental_Init(s_kp, s_ki, s_kd, SERVO_Y_MAX, SERVO_Y_MIN, false, 0.5f);
    pid_x.out = (float)SERVO_X_MID;
    pid_x.last_out = pid_x.out;
    pid_y.out = (float)SERVO_Y_MID;
    pid_y.last_out = pid_y.out;

    prev_angle_x = (float)SERVO_X_MID;
    prev_angle_y = (float)SERVO_Y_MID;

    edge_chasing = false;
    has_prev_det = false;
    interp_step = INTERP_COUNT;
    in_deadzone = false;

    ESP_LOGI(TAG, "Tracking initialized — Kp=%.3f Ki=%.3f Kd=%.3f", s_kp, s_ki, s_kd);
}

// === PID 计算 + 驱动舵机 ===
static void pid_to_servo(float cx, float cy) {
    // 计算误差绝对值
    float err_x = fabsf(cx - SETPOINT_X);
    float err_y = fabsf(cy - SETPOINT_Y);

    // 死区检测
    if (err_x < DEADZONE_PX && err_y < DEADZONE_PX) {
        if (!in_deadzone) {
            in_deadzone = true;
            deadzone_enter_time = esp_timer_get_time();
        } else if (esp_timer_get_time() - deadzone_enter_time > DEADZONE_TIME_US) {
            // 持续在死区内 → 停止更新，清除积分残余
            pid_x.error = 0; pid_x.last_error = 0; pid_x.last_last_error = 0;
            pid_y.error = 0; pid_y.last_error = 0; pid_y.last_last_error = 0;
            return;
        }
    } else {
        in_deadzone = false;
    }

    // 动态增益比例
    float kp_ratio_x = KP_RATIO_SCALE * tanhf(err_x / KP_RATIO_SENS) + KP_RATIO_BASE;
    float kp_ratio_y = KP_RATIO_SCALE * tanhf(err_y / KP_RATIO_SENS) + KP_RATIO_BASE;

    float nxi = err_x / KI_RATIO_SIGMA;
    float nyi = err_y / KI_RATIO_SIGMA;
    float ki_ratio_x = KI_RATIO_SCALE * (1.0f - expf(-nxi * nxi)) + KI_RATIO_BASE;
    float ki_ratio_y = KI_RATIO_SCALE * (1.0f - expf(-nyi * nyi)) + KI_RATIO_BASE;

    // 应用到 PID（基准值 × 比例）
    pid_x.Kp = s_kp * kp_ratio_x;
    pid_x.Ki = s_ki * ki_ratio_x;
    pid_y.Kp = s_kp * kp_ratio_y;
    pid_y.Ki = s_ki * ki_ratio_y;

    float angle_x = PID_Incremental_Calc(&pid_x, cx, SETPOINT_X);
    float angle_y = PID_Incremental_Calc(&pid_y, cy, SETPOINT_Y);

    // 单次增量限幅
    angle_x = clampf(angle_x, prev_angle_x - s_max_increment, prev_angle_x + s_max_increment);
    angle_y = clampf(angle_y, prev_angle_y - s_max_increment, prev_angle_y + s_max_increment);

    // 同步回 PID 累加器
    pid_x.out = angle_x;
    pid_y.out = angle_y;
    prev_angle_x = angle_x;
    prev_angle_y = angle_y;

    servo_set((int16_t)angle_x, (int16_t)angle_y, CTX()->servo.eyelid);
}

// === Core 0 每帧调用：插值 + 边界追逐 ===
void tracking_predict(void) {
    if (!CTX()->flags.tracking_enabled) return;

    // 优先处理插值
    if (interp_step < INTERP_COUNT && has_prev_det) {
        int64_t now = esp_timer_get_time();
        // 每步间隔 = 检测间隔 / (插值数 + 1)
        int64_t step_interval = det_interval / (INTERP_COUNT + 1);
        int64_t next_time = last_det_time + step_interval * (interp_step + 1);

        if (now >= next_time) {
            interp_step++;
            float t = (float)interp_step / (float)(INTERP_COUNT + 1);
            // 线性外推：last_det + velocity * t * det_interval
            float ex_cx = last_cx + vel_px_cx * (float)det_interval * t;
            float ex_cy = last_cy + vel_px_cy * (float)det_interval * t;
            // 限制在画面范围内
            ex_cx = clampf(ex_cx, 0, (float)AI_W);
            ex_cy = clampf(ex_cy, 0, (float)AI_H);
            pid_to_servo(ex_cx, ex_cy);
        }
        return;
    }

    // 边界追逐
    if (!edge_chasing) return;
    if (esp_timer_get_time() - edge_chase_start > EDGE_CHASE_US) {
        edge_chasing = false;
        ESP_LOGI(TAG, "Edge chase timeout");
        return;
    }
    pid_to_servo(edge_chase_cx, edge_chase_cy);
}

// === Core 1 调用：AI 检测到目标后立即算 PID ===
void tracking_correct(float cx, float cy, int x1, int y1, int x2, int y2) {
    if (!CTX()->flags.tracking_enabled) return;

    int64_t now = esp_timer_get_time();

    // 计算速度（像素/微秒）
    if (has_prev_det) {
        int64_t dt = now - last_det_time;
        if (dt > 1000) { // 至少 1ms，防除零
            vel_px_cx = (cx - prev_det_cx) / (float)dt;
            vel_px_cy = (cy - prev_det_cy) / (float)dt;
            det_interval = dt;
        }
    } else {
        vel_px_cx = 0;
        vel_px_cy = 0;
        det_interval = 143000; // 默认约 7fps
    }

    // 保存当前检测
    prev_det_cx = cx;  prev_det_cy = cy;
    last_det_time = now;
    has_prev_det = true;

    // 保存检测框信息（供 target_lost 判断边界）
    last_cx = cx;  last_cy = cy;
    last_x1 = x1;  last_y1 = y1;
    last_x2 = x2;  last_y2 = y2;

    // 检测到目标 → 取消边界追逐，重置插值计数
    edge_chasing = false;
    interp_step = 0;

    pid_to_servo(cx, cy);
}

// === Core 1 调用：AI 未检测到目标 ===
void tracking_target_lost(void) {
    if (!CTX()->flags.tracking_enabled) return;

    // 判断最后一次检测框是否靠近画面边界
    bool near_left   = (last_x1 < EDGE_THRESHOLD);
    bool near_right  = (last_x2 > AI_W - EDGE_THRESHOLD);
    bool near_top    = (last_y1 < EDGE_THRESHOLD);
    bool near_bottom = (last_y2 > AI_H - EDGE_THRESHOLD);

    if (near_left || near_right || near_top || near_bottom) {
        // 启动边界追逐：用最后检测中心继续 PID
        edge_chasing = true;
        edge_chase_cx = last_cx;
        edge_chase_cy = last_cy;
        edge_chase_start = esp_timer_get_time();
        ESP_LOGI(TAG, "Edge chase started (L:%d R:%d T:%d B:%d) → target=(%.0f,%.0f)",
                 near_left, near_right, near_top, near_bottom,
                 edge_chase_cx, edge_chase_cy);
    }
    // 不靠近边界 → 正常丢失，保持当前位置
}

void tracking_set_enabled(bool enabled) {
    CTX()->flags.tracking_enabled = enabled;
    if (enabled) {
        // 同步 PID 累加器到当前舵机位置
        pid_x.out = (float)CTX()->servo.x;
        pid_x.last_out = pid_x.out;
        pid_x.error = 0;
        pid_x.last_error = 0;
        pid_x.last_last_error = 0;

        pid_y.out = (float)CTX()->servo.y;
        pid_y.last_out = pid_y.out;
        pid_y.error = 0;
        pid_y.last_error = 0;
        pid_y.last_last_error = 0;

        prev_angle_x = pid_x.out;
        prev_angle_y = pid_y.out;

        edge_chasing = false;
        has_prev_det = false;
        interp_step = INTERP_COUNT;
    }
    ESP_LOGI(TAG, "Tracking %s", enabled ? "ON" : "OFF");
}

bool tracking_is_enabled(void) {
    return CTX()->flags.tracking_enabled;
}

tracking_params_t tracking_get_params(void) {
    tracking_params_t p;
    p.kp = s_kp;
    p.ki = s_ki;
    p.kd = s_kd;
    p.max_increment = s_max_increment;
    return p;
}

void tracking_set_params(const tracking_params_t *params) {
    s_kp = params->kp;
    s_ki = params->ki;
    s_kd = params->kd;
    s_max_increment = params->max_increment;

    // 热更新 PID 系数
    pid_x.Kp = s_kp;
    pid_x.Ki = s_ki;
    pid_x.Kd = s_kd;
    pid_y.Kp = s_kp;
    pid_y.Ki = s_ki;
    pid_y.Kd = s_kd;

    ESP_LOGI(TAG, "Params updated — Kp=%.3f Ki=%.3f Kd=%.3f MaxInc=%.1f",
             s_kp, s_ki, s_kd, s_max_increment);
}
