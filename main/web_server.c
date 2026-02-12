#include "web_server.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_log.h"

extern QueueHandle_t xWebQueue;
extern volatile bool g_web_connected;
static const char *TAG = "web_server";
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[]   asm("_binary_index_html_gz_end");
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];
    g_web_connected = true;

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if(res != ESP_OK) {
        g_web_connected = false;
        return res;
    }

    while(true) {
        // 等待 app_main 投喂
        if (xQueueReceive(xWebQueue, &fb, pdMS_TO_TICKS(100)) == pdTRUE) {
            res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, (ssize_t)strlen(STREAM_BOUNDARY));
            if (res == ESP_OK) {
                size_t h_len = snprintf(part_buf, 64, STREAM_PART, fb->len);
                res = httpd_resp_send_chunk(req, part_buf, (ssize_t)h_len);
            }
            if (res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)fb->buf, (ssize_t)fb->len);
            }
            esp_camera_fb_return(fb);
            fb = NULL;
        } else {
            // 如果超时没收到帧，检查一下连接是否还需要维持
            // 这里可以发送空包保活，或者不做处理直接继续等
        }
        if(res != ESP_OK) break;
    }

    g_web_connected = false;
    return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
    // 设置 Content-Encoding 为 gzip
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");

    // 设置类型为 HTML
    httpd_resp_set_type(req, "text/html");

    // 发送嵌入的二进制数据
    const size_t index_len = index_html_gz_end - index_html_gz_start;
    return httpd_resp_send(req, (const char *)index_html_gz_start, index_len);
}

esp_err_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting web server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {

        // 注册主页 URI (/)
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &index_uri);

        // 注册流媒体 URI (/stream)
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ESP_FAIL;
}