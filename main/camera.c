#include "camera.h"
#include <esp_log.h>
#include "esp_camera.h"
#define TAG "Camera"

static camera_config_t camera_config = {
    .pin_pwdn  = GPIO_NUM_NC,
    .pin_reset = GPIO_NUM_NC,
    .pin_xclk = GPIO_NUM_15,
    .pin_sccb_sda = GPIO_NUM_4,
    .pin_sccb_scl = GPIO_NUM_5,
    .pin_d7 = GPIO_NUM_16,
    .pin_d6 = GPIO_NUM_17,
    .pin_d5 = GPIO_NUM_18,
    .pin_d4 = GPIO_NUM_12,
    .pin_d3 = GPIO_NUM_10,
    .pin_d2 = GPIO_NUM_8,
    .pin_d1 = GPIO_NUM_9,
    .pin_d0 = GPIO_NUM_11,
    .pin_vsync = GPIO_NUM_6,
    .pin_href = GPIO_NUM_7,
    .pin_pclk = GPIO_NUM_13,
    .sccb_i2c_port = 1,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .jpeg_quality = 8,
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_LATEST
};

esp_err_t camera_init() {
    ESP_LOGI(TAG, "Initializing Camera");
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed");
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    ESP_LOGI(TAG, "Camera initialized");
    return ESP_OK;
}