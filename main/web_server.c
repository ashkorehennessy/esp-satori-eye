#include "web_server.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <sys/param.h>
#include "context.h"

static const char *TAG = "web_server";
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[]   asm("_binary_index_html_gz_end");
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t ota_update_handler(httpd_req_t *req)
{
    if (CTX()->task_bench_c0 != NULL) {
        vTaskDelete(CTX()->task_bench_c0);
        CTX()->task_bench_c0 = NULL;
    }
    if (CTX()->task_bench_c1 != NULL) {
        vTaskDelete(CTX()->task_bench_c1);
        CTX()->task_bench_c1 = NULL;
    }
    if (CTX()->task_ai != NULL) {
        vTaskDelete(CTX()->task_ai);
        CTX()->task_ai = NULL;
    }
    if (CTX()->task_main != NULL) {
        vTaskDelete(CTX()->task_main);
        CTX()->task_main = NULL;
    }
    esp_camera_deinit();
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    char buf[1024];
    int received;
    int remaining = req->content_len;
    ESP_LOGI(TAG, "Starting OTA update...");

    // 1. 获取下一个可用的 OTA 分区
    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Passive OTA partition not found");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, (unsigned int)update_partition->address);

    // 2. 开始 OTA (准备写入)
    // OTA_SIZE_UNKNOWN: 我们边接收边写，不预先校验大小
    if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 3. 循环接收数据并写入 Flash
    while (remaining > 0) {
        // 读取 HTTP 数据
        received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (received <= 0) {
            if (received == HTTPD_SOCK_ERR_TIMEOUT) {
                continue; // 只是超时，重试
            }
            ESP_LOGE(TAG, "File receive failed");
            esp_ota_end(update_handle);
            return ESP_FAIL;
        }

        // 写入 OTA 分区
        if (esp_ota_write(update_handle, buf, received) != ESP_OK) {
            ESP_LOGE(TAG, "Flash write failed");
            esp_ota_end(update_handle);
            return ESP_FAIL;
        }
        remaining -= received;
    }

    // 4. 结束写入并校验
    if (esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // 5. 设置启动分区 (下一次重启生效)
    if (esp_ota_set_boot_partition(update_partition) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OTA Success");
    // 6. 延迟一小会儿让 HTTP 响应发出去，然后重启
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];
    CTX()->flags.is_web_connected = true;

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if(res != ESP_OK) {
        CTX()->flags.is_web_connected = false;
        return res;
    }

    while(true) {
        // 等待 app_main 投喂
        if (xQueueReceive(CTX()->q_camera_frame, &fb, pdMS_TO_TICKS(100)) == pdTRUE) {
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

    CTX()->flags.is_web_connected = false;
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
        // 注册主页
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &index_uri);
        // 注册流媒体
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &stream_uri);
        // 注册 OTA 更新接口
        httpd_uri_t ota_uri = {
            .uri       = "/update",
            .method    = HTTP_POST,
            .handler   = ota_update_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &ota_uri);

        return ESP_OK;
    }

    ESP_LOGE(TAG, "Error starting server!");
    return ESP_FAIL;
}