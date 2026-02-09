#include "web_server.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "web_server";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if(res != ESP_OK) {
        return res;
    }

    while(true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        uint8_t* _jpg_buf = fb->buf;
        size_t _jpg_buf_len = fb->len;

        char part_buf[64];

        // A. 发送边界
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, (ssize_t)strlen(STREAM_BOUNDARY));

        // B. 发送头部信息
        if(res == ESP_OK) {
            size_t h_len = snprintf(part_buf, 64, STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, part_buf, (ssize_t)h_len);
        }

        // C. 发送图片数据
        if(res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, (ssize_t)_jpg_buf_len);
        }

        esp_camera_fb_return(fb);
        fb = NULL;

        if(res != ESP_OK) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return res;
}

esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.stack_size = 4096;

    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);

        ESP_LOGI(TAG, "Web Server started successfully");
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ESP_FAIL;
}