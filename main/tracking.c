#include "tracking.h"
#include "PID.h"
#include "context.h"
#include "servo.h"
#include "ai.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "Tracking";

// =============================================
// 运行时可调参数（默认值，可通过 Web UI 修改）
// =============================================
static float s_kp = 0.07f;
static float s_ki = 0.03f;
static float s_kd = 0.0f;
static float s_max_increment = 90.0f;

// 舵机方向反转标志（true = 画面右移时舵机角度减小）
#define INVERT_X   true
#define INVERT_Y   true

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

// =============================================
// 运动预测器参数
// =============================================
#define VEL_SMOOTH_ALPHA  0.5f        // 速度指数平滑系数（越大越灵敏）
#define TARGET_LOST_US    500000      // 兜底超时 500ms（正常走 target_lost 立停）

// =============================================

static PID_Incremental pid_x;
static PID_Incremental pid_y;

// PID 增量限幅
static float prev_angle_x;
static float prev_angle_y;

// 运动预测器状态
static float pred_cx, pred_cy;          // 预测位置（画面坐标 0~240）
static float vel_x, vel_y;             // 速度（像素 / 微秒）
static int64_t last_predict_time;       // 上次 predict 时间戳
static int64_t last_correct_time;       // 上次 correct 时间戳
static volatile bool has_target;        // 是否有跟踪目标

// 限幅辅助
static inline float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void tracking_init(void) {
    // 增量式 PID，输出范围为实际舵机行程
    pid_x = PID_Incremental_Init(s_kp, s_ki, s_kd,
                                  SERVO_X_MAX, SERVO_X_MIN,
                                  true, 0.5f);
    pid_y = PID_Incremental_Init(s_kp, s_ki, s_kd,
                                  SERVO_Y_MAX, SERVO_Y_MIN,
                                  true, 0.5f);
    // PID 累加器初始化到舵机中位
    pid_x.out = (float)SERVO_X_MID;
    pid_x.last_out = pid_x.out;
    pid_y.out = (float)SERVO_Y_MID;
    pid_y.last_out = pid_y.out;

    prev_angle_x = (float)SERVO_X_MID;
    prev_angle_y = (float)SERVO_Y_MID;

    // 预测器初始化
    pred_cx = SETPOINT_X;
    pred_cy = SETPOINT_Y;
    vel_x = 0;
    vel_y = 0;
    last_predict_time = esp_timer_get_time();
    last_correct_time = last_predict_time;
    has_target = false;

    ESP_LOGI(TAG, "Tracking initialized — Kp=%.2f Ki=%.3f Kd=%.2f InvX=%d InvY=%d",
             s_kp, s_ki, s_kd, INVERT_X, INVERT_Y);
}

// === Core 0 调用：每帧预测 + PID → servo ===
void tracking_predict(void) {
    if (!CTX()->flags.tracking_enabled) return;
    if (!has_target) return;  // 无目标时保持不动

    int64_t now = esp_timer_get_time();

    // 兜底超时检查（正常情况下 target_lost() 会更早触发）
    if (now - last_correct_time > TARGET_LOST_US) {
        has_target = false;
        ESP_LOGW(TAG, "Target lost (timeout)");
        return;
    }

    // 时间差（微秒）
    float dt = (float)(now - last_predict_time);
    last_predict_time = now;

    // 外推预测位置
    pred_cx += vel_x * dt;
    pred_cy += vel_y * dt;

    // 限制在画面范围内
    pred_cx = clampf(pred_cx, 0, (float)AI_W);
    pred_cy = clampf(pred_cy, 0, (float)AI_H);

    // 方向处理
    float input_x = INVERT_X ? (AI_W - pred_cx) : pred_cx;
    float input_y = INVERT_Y ? (AI_H - pred_cy) : pred_cy;

    // PID 计算
    float angle_x = PID_Incremental_Calc(&pid_x, input_x, SETPOINT_X);
    float angle_y = PID_Incremental_Calc(&pid_y, input_y, SETPOINT_Y);

    // 单次增量限幅
    angle_x = clampf(angle_x, prev_angle_x - s_max_increment, prev_angle_x + s_max_increment);
    angle_y = clampf(angle_y, prev_angle_y - s_max_increment, prev_angle_y + s_max_increment);

    // 同步回 PID 累加器
    pid_x.out = angle_x;
    pid_y.out = angle_y;
    prev_angle_x = angle_x;
    prev_angle_y = angle_y;

    // 驱动舵机
    servo_set((int16_t)angle_x, (int16_t)angle_y, CTX()->servo.eyelid);
}

// === Core 1 调用：AI 检测到目标后校正 ===
void tracking_correct(float cx, float cy) {
    if (!CTX()->flags.tracking_enabled) return;

    int64_t now = esp_timer_get_time();

    if (has_target) {
        // 计算从上次校正到现在的时间
        float dt = (float)(now - last_correct_time);
        if (dt > 1000) {  // 至少 1ms，防除零
            // 新速度 = (真实位置 - 预测位置) / dt
            // 注意：用真实位置与预测位置的差，因为预测位置已经在外推了
            // 如果预测准确，差值≈0，速度基本不变
            // 如果预测偏了，差值会修正速度方向
            float new_vel_x = (cx - pred_cx) / dt;
            float new_vel_y = (cy - pred_cy) / dt;

            // 指数平滑
            vel_x = VEL_SMOOTH_ALPHA * new_vel_x + (1.0f - VEL_SMOOTH_ALPHA) * vel_x;
            vel_y = VEL_SMOOTH_ALPHA * new_vel_y + (1.0f - VEL_SMOOTH_ALPHA) * vel_y;
        }
    } else {
        // 首次获得目标，速度归零
        vel_x = 0;
        vel_y = 0;
    }

    // 强制校正位置到真实检测值
    pred_cx = cx;
    pred_cy = cy;
    last_correct_time = now;
    has_target = true;
}

// === Core 1 调用：AI 未检测到目标 ===
void tracking_target_lost(void) {
    if (!has_target) return;
    has_target = false;
    vel_x = 0;
    vel_y = 0;
    ESP_LOGI(TAG, "Target lost (AI: count=0)");
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

        // 预测器重置
        pred_cx = SETPOINT_X;
        pred_cy = SETPOINT_Y;
        vel_x = 0;
        vel_y = 0;
        last_predict_time = esp_timer_get_time();
        last_correct_time = last_predict_time;
        has_target = false;
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
