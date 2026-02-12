#ifndef ESP_SATORI_EYE_CONTENT_H
#define ESP_SATORI_EYE_CONTENT_H
#include <stdbool.h>
#include "freertos/FreeRTOS.h"

// 系统状态标志位
typedef struct {
    volatile bool is_web_connected;
    volatile bool is_bench_running;
} sys_flags_t;

typedef struct {
    uint8_t *buf_a;    // Ping Buffer
    uint8_t *buf_b;    // Pong Buffer
    int buf_selector;  // 当前正在写入哪个 Buffer
} ai_runtime_t;

// 核心上下文结构体
typedef struct {
    sys_flags_t flags;
    ai_runtime_t ai;

    TaskHandle_t task_main;       // 主任务句柄
    TaskHandle_t task_ai;         // AI 任务句柄
    TaskHandle_t task_bench_c0;   // 跑分任务句柄0
    TaskHandle_t task_bench_c1;   // 跑分任务句柄1

    QueueHandle_t q_camera_frame;
    QueueHandle_t q_ai_inference;  // 通信队列：Core 0 告诉 Core 1 哪个 Buffer 准备好了


    SemaphoreHandle_t mutex;
} context_t;

extern context_t g_app_ctx;

static inline __attribute__((always_inline)) context_t* CTX(void) {
    return &g_app_ctx;
}
void context_init(void);

#endif //ESP_SATORI_EYE_CONTENT_H