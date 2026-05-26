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

// 初始化追踪模块（创建 PID 实例）
void tracking_init(void);

// Core 0 每帧调用：边界丢失时继续追踪
void tracking_predict(void);

// Core 1 检测完成后调用：用检测框中心跑 PID → 驱动舵机
// x1,y1,x2,y2 为检测框边界（用于边界丢失判断）
void tracking_correct(float cx, float cy, int x1, int y1, int x2, int y2);

// Core 1 检测到 count=0 时调用（边界丢失时触发追逐）
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
