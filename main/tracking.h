#ifndef ESP_SATORI_EYE_TRACKING_H
#define ESP_SATORI_EYE_TRACKING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PID 运行时参数（可通过 Web 动态调整）
typedef struct {
    float kp;
    float ki;
    float kd;
    float max_increment;
} tracking_params_t;

// 初始化追踪模块
void tracking_init(void);

// === 双速率追踪接口 ===

// Core 0 每帧调用：用预测位置跑 PID → 舵机
void tracking_predict(void);

// Core 1 检测完成后调用：校正预测器 + 标记需要更新模板
void tracking_correct(float cx, float cy);

// Core 1 检测到 count=0：立即停止
void tracking_target_lost(void);

// === SAD 模板匹配接口 ===

// 从 RGB buffer 的检测框区域提取 16×16 灰度模板
void tracking_extract_template(const uint8_t *rgb_buf);

// 在 RGB buffer 中搜索模板，成功则调用 tracking_correct()
void tracking_match(const uint8_t *rgb_buf);

// 是否有可用模板
bool tracking_has_template(void);

// 是否需要更新模板（新 AI 检测到达）
bool tracking_needs_template_update(void);

// === 开关与参数 ===

void tracking_set_enabled(bool enabled);
bool tracking_is_enabled(void);

tracking_params_t tracking_get_params(void);
void tracking_set_params(const tracking_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // ESP_SATORI_EYE_TRACKING_H
