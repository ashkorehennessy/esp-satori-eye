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
    // start_ai();
    start_benchmark();
    ESP_LOGI(TAG, "System Ready! Connect to WiFi 'Satori-Eye' and visit http://192.168.4.1/stream");
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(1); continue; }
        ESP_LOGI("CAM_DEBUG", "JPEG Size: %zu Bytes", fb->len);

        // AI 分支
        // if (uxQueueSpacesAvailable(CTX()->q_ai_inference) > 0) {
        //     uint8_t *target_buf = CTX()->ai.buf_selector == 0 ? CTX()->ai.buf_a : CTX()->ai.buf_b;
        //     ai_decode_jpeg_wrapper(fb->buf, fb->len, target_buf);
        //     xQueueSend(CTX()->q_ai_inference, &target_buf, 0);
        //     CTX()->ai.buf_selector = CTX()->ai.buf_selector;
        // }

        // Web 分支
        bool frame_taken_by_web = false;

        if (CTX()->flags.is_web_connected) {
            // 尝试把指针塞给 Web
            // 1. 如果 Web 正在忙（队列满），xQueueSend 失败 -> 我们自己回收 fb
            // 2. 如果 Web 空闲，xQueueSend 成功 -> 责任移交给 Web，它发完负责回收
            if (xQueueSend(CTX()->q_camera_frame, &fb, 0) == pdTRUE) {
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

