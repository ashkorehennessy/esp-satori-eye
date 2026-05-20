#ifndef ESP_SATORI_EYE_TRACKING_H
#define ESP_SATORI_EYE_TRACKING_H

#include <stdbool.h>

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

// 初始化追踪模块（创建 PID 实例 + 预测器）
void tracking_init(void);

// === 双速率追踪接口 ===

// Core 0 每帧调用（~30 FPS）：用预测位置跑 PID → 驱动舵机
void tracking_predict(void);

// Core 1 检测完成后调用：用真实检测框校正预测器位置和速度
void tracking_correct(float cx, float cy);

// Core 1 检测到 count=0 时调用：立即停止预测
void tracking_target_lost(void);

// === 开关与参数 ===

void tracking_set_enabled(bool enabled);
bool tracking_is_enabled(void);

tracking_params_t tracking_get_params(void);
void tracking_set_params(const tracking_params_t *params);

#ifdef __cplusplus
}
#endif

#endif // ESP_SATORI_EYE_TRACKING_H
