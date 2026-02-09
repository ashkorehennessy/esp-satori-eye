#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "camera.h"
#define TAG "app_main"

void app_main(void)
{
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "System Halted");
        return;
    }
    while (1) {
        camera_fb_t *pic = esp_camera_fb_get();
        if (pic) {
            ESP_LOGI("Camera", "Picture taken! Size: %zu bytes, Width: %d, Height: %d",
                     pic->len, pic->width, pic->height);

            esp_camera_fb_return(pic);
        } else {
            ESP_LOGE("Camera", "Failed to capture frame!");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

