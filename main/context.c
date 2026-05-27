#include "context.h"
#include <esp_camera.h>
#include "esp_log.h"
#include "ai.h"

context_t g_app_ctx = {0};

void context_init(void) {
    g_app_ctx.q_camera_frame = xQueueCreate(1, sizeof(camera_fb_t *));
    g_app_ctx.q_ai_inference = xQueueCreate(2, sizeof(uint8_t *));

    g_app_ctx.ai.buf_a = heap_caps_malloc(AI_RGB_SIZE, MALLOC_CAP_SPIRAM);
    g_app_ctx.ai.buf_b = heap_caps_malloc(AI_RGB_SIZE, MALLOC_CAP_SPIRAM);
    g_app_ctx.ai.buf_selector = 0;
    if (!g_app_ctx.ai.buf_a || !g_app_ctx.ai.buf_b) {
        ESP_LOGE("CTX", "CRITICAL: Failed to allocate AI buffers in SPIRAM!");
    }

    g_app_ctx.flags.is_web_connected = false;
    g_app_ctx.flags.is_bench_running = true;
    g_app_ctx.flags.ai_enabled = true;
    g_app_ctx.flags.tracking_enabled = true;

    // 舵机初始位置：居中
    g_app_ctx.servo.x = 90;
    g_app_ctx.servo.y = 90;
    g_app_ctx.servo.eyelid = 90;

    // 检测结果清零
    g_app_ctx.detections.count = 0;
    g_app_ctx.detections.timestamp = 0;

    g_app_ctx.mutex = xSemaphoreCreateMutex();

    ESP_LOGI("CTX", "System Context Initialized");
}
