#include "tracking.h"
#include "PID.h"
#include "context.h"
#include "servo.h"
#include "ai.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdlib.h>

static const char *TAG = "Tracking";

// =============================================
// 运行时可调参数（默认值，可通过 Web UI 修改）
// =============================================
static float s_kp = 0.07f;
static float s_ki = 0.07f;
static float s_kd = 0.0f;
static float s_max_increment = 90.0f;

// 舵机方向反转标志
#define INVERT_X   true
#define INVERT_Y   true

// 设定点：画面中心
#define SETPOINT_X  ((float)AI_W / 2.0f)
#define SETPOINT_Y  ((float)AI_H / 2.0f)

// 舵机行程限位
#define SERVO_X_MIN   65
#define SERVO_X_MAX   135
#define SERVO_X_MID   100
#define SERVO_Y_MIN   65
#define SERVO_Y_MAX   120
#define SERVO_Y_MID   90

// =============================================
// SAD 模板匹配参数
// =============================================
#define TMPL_SIZE       16           // 模板尺寸 16×16
#define SEARCH_RADIUS   20           // 搜索半径（像素）
#define SEARCH_STRIDE   2            // 搜索步长
#define SAD_THRESHOLD   (TMPL_SIZE * TMPL_SIZE * 35)  // 每像素平均差>35视为失败

// 运动预测器参数
#define VEL_SMOOTH_ALPHA  0.3f
#define TARGET_LOST_US    500000

// =============================================

static PID_Incremental pid_x;
static PID_Incremental pid_y;
static float prev_angle_x;
static float prev_angle_y;

// 运动预测器状态
static float pred_cx, pred_cy;
static float vel_x, vel_y;
static int64_t last_predict_time;
static int64_t last_correct_time;
static volatile bool has_target;

// SAD 模板匹配状态
static uint8_t s_template[TMPL_SIZE * TMPL_SIZE];   // 256 bytes
static bool s_has_template = false;
static volatile bool s_need_template_update = false; // Core 1 → Core 0 信号
static int s_last_box_x1, s_last_box_y1, s_last_box_x2, s_last_box_y2;  // 最近检测框

// 限幅辅助
static inline float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

// 从 RGB buffer 读取灰度值（取绿色通道，避免额外转换）
static inline uint8_t rgb_to_gray(const uint8_t *rgb_buf, int x, int y) {
    return rgb_buf[(y * AI_W + x) * 3 + 1];  // Green channel
}

void tracking_init(void) {
    pid_x = PID_Incremental_Init(s_kp, s_ki, s_kd,
                                  SERVO_X_MAX, SERVO_X_MIN,
                                  true, 0.5f);
    pid_y = PID_Incremental_Init(s_kp, s_ki, s_kd,
                                  SERVO_Y_MAX, SERVO_Y_MIN,
                                  true, 0.5f);
    pid_x.out = (float)SERVO_X_MID;
    pid_x.last_out = pid_x.out;
    pid_y.out = (float)SERVO_Y_MID;
    pid_y.last_out = pid_y.out;

    prev_angle_x = (float)SERVO_X_MID;
    prev_angle_y = (float)SERVO_Y_MID;

    pred_cx = SETPOINT_X;
    pred_cy = SETPOINT_Y;
    vel_x = 0;
    vel_y = 0;
    last_predict_time = esp_timer_get_time();
    last_correct_time = last_predict_time;
    has_target = false;
    s_has_template = false;
    s_need_template_update = false;

    ESP_LOGI(TAG, "Tracking initialized — Kp=%.2f Ki=%.3f Kd=%.2f",
             s_kp, s_ki, s_kd);
}

// =============================================
// SAD 模板匹配
// =============================================

void tracking_extract_template(const uint8_t *rgb_buf) {
    // 使用最近的检测框坐标
    int bx1 = s_last_box_x1, by1 = s_last_box_y1;
    int bx2 = s_last_box_x2, by2 = s_last_box_y2;
    int box_cx = (bx1 + bx2) / 2;
    int box_cy = (by1 + by2) / 2;

    // 从检测框中心裁切 16×16 灰度 patch
    int start_x = box_cx - TMPL_SIZE / 2;
    int start_y = box_cy - TMPL_SIZE / 2;

    // 边界保护
    if (start_x < 0) start_x = 0;
    if (start_y < 0) start_y = 0;
    if (start_x + TMPL_SIZE > AI_W) start_x = AI_W - TMPL_SIZE;
    if (start_y + TMPL_SIZE > AI_H) start_y = AI_H - TMPL_SIZE;

    for (int ty = 0; ty < TMPL_SIZE; ty++) {
        for (int tx = 0; tx < TMPL_SIZE; tx++) {
            s_template[ty * TMPL_SIZE + tx] = rgb_to_gray(rgb_buf, start_x + tx, start_y + ty);
        }
    }

    s_has_template = true;
    s_need_template_update = false;
    ESP_LOGI(TAG, "Template extracted at (%d,%d) from box [%d,%d,%d,%d]",
             box_cx, box_cy, bx1, by1, bx2, by2);
}

void tracking_match(const uint8_t *rgb_buf) {
    if (!s_has_template || !has_target) return;

    // 搜索中心 = 当前预测位置
    int search_cx = (int)pred_cx;
    int search_cy = (int)pred_cy;

    uint32_t best_sad = UINT32_MAX;
    int best_x = search_cx;
    int best_y = search_cy;

    // 搜索范围
    int sx_min = search_cx - SEARCH_RADIUS;
    int sy_min = search_cy - SEARCH_RADIUS;
    int sx_max = search_cx + SEARCH_RADIUS;
    int sy_max = search_cy + SEARCH_RADIUS;

    // 边界保护（模板不能超出画面）
    int half = TMPL_SIZE / 2;
    if (sx_min < half) sx_min = half;
    if (sy_min < half) sy_min = half;
    if (sx_max > AI_W - half) sx_max = AI_W - half;
    if (sy_max > AI_H - half) sy_max = AI_H - half;

    for (int cy = sy_min; cy <= sy_max; cy += SEARCH_STRIDE) {
        for (int cx = sx_min; cx <= sx_max; cx += SEARCH_STRIDE) {
            uint32_t sad = 0;
            int px_start = cx - half;
            int py_start = cy - half;

            for (int ty = 0; ty < TMPL_SIZE; ty++) {
                for (int tx = 0; tx < TMPL_SIZE; tx++) {
                    int diff = (int)rgb_to_gray(rgb_buf, px_start + tx, py_start + ty)
                             - (int)s_template[ty * TMPL_SIZE + tx];
                    sad += (uint32_t)abs(diff);
                }
            }

            if (sad < best_sad) {
                best_sad = sad;
                best_x = cx;
                best_y = cy;
            }
        }
    }

    // 判断匹配质量
    if (best_sad < SAD_THRESHOLD) {
        // 匹配成功：用匹配位置校正预测器
        tracking_correct((float)best_x, (float)best_y);
    }
    // 匹配失败：不更新，靠速度预测器惯性滑行
}

bool tracking_has_template(void) {
    return s_has_template;
}

bool tracking_needs_template_update(void) {
    return s_need_template_update;
}

// =============================================
// 运动预测 + PID 控制
// =============================================

void tracking_predict(void) {
    if (!CTX()->flags.tracking_enabled) return;
    if (!has_target) return;

    int64_t now = esp_timer_get_time();

    // 兜底超时
    if (now - last_correct_time > TARGET_LOST_US) {
        has_target = false;
        ESP_LOGW(TAG, "Target lost (timeout)");
        return;
    }

    float dt = (float)(now - last_predict_time);
    last_predict_time = now;

    // 外推预测
    pred_cx += vel_x * dt;
    pred_cy += vel_y * dt;
    pred_cx = clampf(pred_cx, 0, (float)AI_W);
    pred_cy = clampf(pred_cy, 0, (float)AI_H);

    // 方向处理 + PID
    float input_x = INVERT_X ? (AI_W - pred_cx) : pred_cx;
    float input_y = INVERT_Y ? (AI_H - pred_cy) : pred_cy;

    float angle_x = PID_Incremental_Calc(&pid_x, input_x, SETPOINT_X);
    float angle_y = PID_Incremental_Calc(&pid_y, input_y, SETPOINT_Y);

    angle_x = clampf(angle_x, prev_angle_x - s_max_increment, prev_angle_x + s_max_increment);
    angle_y = clampf(angle_y, prev_angle_y - s_max_increment, prev_angle_y + s_max_increment);

    pid_x.out = angle_x;
    pid_y.out = angle_y;
    prev_angle_x = angle_x;
    prev_angle_y = angle_y;

    servo_set((int16_t)angle_x, (int16_t)angle_y, CTX()->servo.eyelid);
}

void tracking_correct(float cx, float cy) {
    if (!CTX()->flags.tracking_enabled) return;

    int64_t now = esp_timer_get_time();

    if (has_target) {
        float dt = (float)(now - last_correct_time);
        if (dt > 1000) {
            float new_vel_x = (cx - pred_cx) / dt;
            float new_vel_y = (cy - pred_cy) / dt;
            vel_x = VEL_SMOOTH_ALPHA * new_vel_x + (1.0f - VEL_SMOOTH_ALPHA) * vel_x;
            vel_y = VEL_SMOOTH_ALPHA * new_vel_y + (1.0f - VEL_SMOOTH_ALPHA) * vel_y;
        }
    } else {
        vel_x = 0;
        vel_y = 0;
    }

    pred_cx = cx;
    pred_cy = cy;
    last_correct_time = now;
    has_target = true;

    // 如果有最新检测框，保存坐标并标记需要更新模板
    if (CTX()->detections.count > 0) {
        detection_t *d = &CTX()->detections.items[0];
        s_last_box_x1 = d->x1;
        s_last_box_y1 = d->y1;
        s_last_box_x2 = d->x2;
        s_last_box_y2 = d->y2;
        s_need_template_update = true;
    }
}

void tracking_target_lost(void) {
    if (!has_target) return;
    has_target = false;
    vel_x = 0;
    vel_y = 0;
    // 模板保留，目标重新出现时可继续匹配
    ESP_LOGI(TAG, "Target lost (AI: count=0)");
}

// =============================================
// 开关与参数
// =============================================

void tracking_set_enabled(bool enabled) {
    CTX()->flags.tracking_enabled = enabled;
    if (enabled) {
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

        pred_cx = SETPOINT_X;
        pred_cy = SETPOINT_Y;
        vel_x = 0;
        vel_y = 0;
        last_predict_time = esp_timer_get_time();
        last_correct_time = last_predict_time;
        has_target = false;
        s_has_template = false;
        s_need_template_update = false;
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

    pid_x.Kp = s_kp;  pid_x.Ki = s_ki;  pid_x.Kd = s_kd;
    pid_y.Kp = s_kp;  pid_y.Ki = s_ki;  pid_y.Kd = s_kd;

    ESP_LOGI(TAG, "Params updated — Kp=%.3f Ki=%.3f Kd=%.3f MaxInc=%.1f",
             s_kp, s_ki, s_kd, s_max_increment);
}
