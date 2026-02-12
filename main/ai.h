#ifndef ESP_SATORI_EYE_AI_H
#define ESP_SATORI_EYE_AI_H
#include "freertos/FreeRTOS.h"

// AI 模型输入参数
#define AI_W 240
#define AI_H 240
#define AI_RGB_SIZE (AI_W * AI_H * 3)

#ifdef __cplusplus
extern "C" {
#endif

    // 初始化 AI
    void start_ai(void);

    // Core 0调用的解码函数
    // 参数: jpg数据, jpg长度, 解码结果存放的目标buffer指针
    void ai_decode_jpeg_wrapper(uint8_t *jpg_data, size_t jpg_len, uint8_t *target_rgb_buf);

    // AI 推理任务
    void ai_inference_task(void *arg);

#ifdef __cplusplus
}
#endif
#endif //ESP_SATORI_EYE_AI_H