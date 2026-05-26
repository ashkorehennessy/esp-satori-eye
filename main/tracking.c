#include "tracking.h"
#include "PID.h"
#include "context.h"
#include "servo.h"
#include "ai.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "Tracking";

// =============================================
// 运行时可调参数（默认值，可通过 Web UI 修改）
// =============================================
static float s_kp = 0.07f;
static float s_ki = 0.03f;
static float s_kd = 0.0f;
static float s_max_increment = 90.0f;

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

void tracking_init(void) {
    // 增量式 PID，输出范围为实际舵机行程
    pid_x = PID_Incremental_Init(s_kp, s_ki, s_kd, SERVO_X_MAX, SERVO_X_MIN, false, 0.5f);
    pid_y = PID_Incremental_Init(s_kp, s_ki, s_kd, SERVO_Y_MAX, SERVO_Y_MIN, false, 0.5f);
    // PID 累加器初始化到舵机中位
    pid_x.out = (float)SERVO_X_MID;
    pid_x.last_out = pid_x.out;
    pid_y.out = (float)SERVO_Y_MID;
    pid_y.last_out = pid_y.out;

    prev_angle_x = (float)SERVO_X_MID;
    prev_angle_y = (float)SERVO_Y_MID;

    ESP_LOGI(TAG, "Tracking initialized — Kp=%.3f Ki=%.3f Kd=%.3f", s_kp, s_ki, s_kd);
}

// === 动态 PID 增益调度 ===
// ratio = scale * tanh(|error| / sensitivity) + base
// 目标靠近中心 → ratio 小 → 减少震荡
// 目标远离中心 → ratio 大 → 加快响应
#define KP_RATIO_SCALE  0.7f
#define KP_RATIO_BASE   0.3f
#define KP_RATIO_SENS   60.0f

#define KI_RATIO_SCALE  0.7f
#define KI_RATIO_BASE   0.3f
#define KI_RATIO_SENS   65.0f

// === PID 计算 + 驱动舵机 ===
static void pid_to_servo(float cx, float cy) {
    // 计算误差绝对值
    float err_x = fabsf(cx - SETPOINT_X);
    float err_y = fabsf(cy - SETPOINT_Y);

    // 动态增益比例
    float kp_ratio_x = KP_RATIO_SCALE * tanhf(err_x / KP_RATIO_SENS) + KP_RATIO_BASE;
    float ki_ratio_x = KI_RATIO_SCALE * tanhf(err_x / KI_RATIO_SENS) + KI_RATIO_BASE;
    float kp_ratio_y = KP_RATIO_SCALE * tanhf(err_y / KP_RATIO_SENS) + KP_RATIO_BASE;
    float ki_ratio_y = KI_RATIO_SCALE * tanhf(err_y / KI_RATIO_SENS) + KI_RATIO_BASE;

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

// === Core 0 每帧调用（保留接口，当前为空操作）===
void tracking_predict(void) {
    // 直接 PID 模式下不需要预测，PID 在 correct 中执行
}

// === Core 1 调用：AI 检测到目标后立即算 PID ===
void tracking_correct(float cx, float cy) {
    if (!CTX()->flags.tracking_enabled) return;
    pid_to_servo(cx, cy);
}

// === Core 1 调用：AI 未检测到目标 ===
void tracking_target_lost(void) {
    // 丢失目标 = 保持当前位置，无需操作
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
