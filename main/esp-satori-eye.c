#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_chip_info.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "context.h"
#include "camera.h"
#include "wifi.h"
#include "web_server.h"
#include "ai.h"
#include "servo.h"
#include "tracking.h"
#define TAG "app_main"
void start_benchmark(void);
void app_main(void)
{
    context_init();
    CTX()->task_main = xTaskGetCurrentTaskHandle();
    const esp_partition_t *running = esp_ota_get_running_partition();
    const char *slot_char = running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0 ? "A" : "B";
    ESP_LOGI(TAG, "Current Slot     : %s (%s)", slot_char, running->label);
    ESP_LOGI(TAG, "Address Offset   : 0x%06x", (unsigned int)running->address);
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "System Halted");
        return;
    }
    wifi_init_softap();
    start_webserver();
    servo_init();
    tracking_init();
    start_ai();
    start_benchmark();
    ESP_LOGI(TAG, "System Ready! Connect to WiFi 'Satori-Eye' and visit http://192.168.4.1");
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(1); continue; }

        // AI 分支：将 JPEG 解码到 Ping-Pong Buffer，送入 AI 推理队列
        if (CTX()->flags.ai_enabled && uxQueueSpacesAvailable(CTX()->q_ai_inference) > 0) {
            uint8_t *target_buf = CTX()->ai.buf_selector == 0 ? CTX()->ai.buf_a : CTX()->ai.buf_b;
            ai_decode_jpeg_wrapper(fb->buf, fb->len, target_buf);

            // 追踪：AI 检测到新目标后，从当前解码帧提取模板（buffer 尚未移交 Core 1）
            if (CTX()->flags.tracking_enabled && tracking_needs_template_update()) {
                tracking_extract_template(target_buf);
            }

            xQueueSend(CTX()->q_ai_inference, &target_buf, 0);
            CTX()->ai.buf_selector = 1 - CTX()->ai.buf_selector;
        }
        // 追踪分支：AI 队列满（Core 1 忙），复用当前 target buf 解码做 SAD 匹配
        // buf_selector 指向的 buffer 是"下一个要写的"，尚未提交给 AI，可安全使用
        else if (CTX()->flags.tracking_enabled && tracking_has_template()) {
            uint8_t *track_buf = CTX()->ai.buf_selector == 0 ? CTX()->ai.buf_a : CTX()->ai.buf_b;
            ai_decode_jpeg_wrapper(fb->buf, fb->len, track_buf);
            tracking_match(track_buf);
        }

        // Web 分支
        bool frame_taken_by_web = false;

        if (CTX()->flags.is_web_connected) {
            if (xQueueSend(CTX()->q_camera_frame, &fb, 0) == pdTRUE) {
                frame_taken_by_web = true;
            }
        }

        if (!frame_taken_by_web) {
            esp_camera_fb_return(fb);
        }

        // 快速追踪：每帧用预测位置跑 PID → 舵机
        tracking_predict();

        // 喂狗
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

