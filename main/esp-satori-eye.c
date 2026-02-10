#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "camera.h"
#include "wifi.h"
#include "web_server.h"
#include "ai.h"
#define TAG "app_main"
volatile bool g_web_connected;
QueueHandle_t xWebQueue;
int buffer_selector = 0;
void start_benchmark(void);
void app_main(void)
{
    xWebQueue = xQueueCreate(1, sizeof(camera_fb_t *));
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "System Halted");
        return;
    }
    wifi_init_softap();
    ai_init_service();
    start_benchmark();
    xTaskCreatePinnedToCore(ai_inference_task, "AI_Task", 8192, NULL, 5, NULL, 1);
    start_webserver();
    ESP_LOGI(TAG, "System Ready! Connect to WiFi 'Satori-Eye' and visit http://192.168.4.1/stream");
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(1); continue; }

        // AI 分支
        if (uxQueueSpacesAvailable(xAIQueue) > 0) {
            uint8_t *target_buf = buffer_selector == 0 ? g_ai_buf_a : g_ai_buf_b;
            ai_decode_jpeg_wrapper(fb->buf, fb->len, target_buf);
            xQueueSend(xAIQueue, &target_buf, 0);
            buffer_selector = !buffer_selector;
        }

        // Web 分支
        bool frame_taken_by_web = false;

        if (g_web_connected) {
            // 尝试把指针塞给 Web
            // 1. 如果 Web 正在忙（队列满），xQueueSend 失败 -> 我们自己回收 fb
            // 2. 如果 Web 空闲，xQueueSend 成功 -> 责任移交给 Web，它发完负责回收
            if (xQueueSend(xWebQueue, &fb, 0) == pdTRUE) {
                frame_taken_by_web = true;
            }
        }

        // 只有当 Web 没拿走 fb 时才回收
        if (!frame_taken_by_web) {
            esp_camera_fb_return(fb);
        }

        // 喂狗
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

