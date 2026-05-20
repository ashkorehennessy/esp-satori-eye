#ifndef ESP_SATORI_EYE_CONTENT_H
#define ESP_SATORI_EYE_CONTENT_H
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

// 系统状态标志位
typedef struct {
    volatile bool is_web_connected;
    volatile bool is_bench_running;
    volatile bool ai_enabled;      // AI 推理开关（Web UI 可控）
    volatile bool tracking_enabled; // 自动追踪开关（Web UI 可控）
} sys_flags_t;

typedef struct {
    uint8_t *buf_a;    // Ping Buffer
    uint8_t *buf_b;    // Pong Buffer
    int buf_selector;  // 当前正在写入哪个 Buffer
} ai_runtime_t;

// 舵机目标位置（Web UI → ESP32）
typedef struct {
    volatile int16_t x;       // 水平轴，0~180°
    volatile int16_t y;       // 垂直轴，0~180°
    volatile int16_t eyelid;  // 眼皮轴，0~180°
} servo_target_t;

// AI 检测结果（AI Task → Web UI）
#define MAX_DETECTIONS 4
typedef struct {
    int x1, y1, x2, y2;  // 边界框像素坐标（240x240 空间）
    int category;
    float score;
} detection_t;

typedef struct {
    detection_t items[MAX_DETECTIONS];
    volatile int count;           // 当前检测到的目标数
    volatile int64_t timestamp;   // 最近一次推理的时间戳 (us)
} detection_results_t;

// 核心上下文结构体
typedef struct {
    sys_flags_t flags;
    ai_runtime_t ai;
    servo_target_t servo;
    detection_results_t detections;

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