#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "camera.h"
#include "wifi.h"
#include "web_server.h"
#define TAG "app_main"
void start_benchmark(void);
void app_main(void)
{
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "System Halted");
        return;
    }
    wifi_init_softap();
    start_benchmark();
    start_webserver();
    ESP_LOGI(TAG, "System Ready! Connect to WiFi 'Satori-Eye' and visit http://192.168.4.1/stream");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

