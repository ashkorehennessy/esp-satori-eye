#include "web_server.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include <sys/param.h>
#include <stdlib.h>
#include <string.h>
#include "context.h"
#include "servo.h"

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
    // 单客户端限制：如果已有连接，拒绝新请求
    if (CTX()->flags.is_web_connected) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "Stream already in use by another client");
        return ESP_FAIL;
    }

    esp_err_t res = ESP_OK;
    char part_buf[64];
    CTX()->flags.is_web_connected = true;

    // 预分配 JPEG 拷贝缓冲区（PSRAM），用于快速归还 camera fb
    const size_t jpeg_buf_size = 64 * 1024; // 64KB，足够 240x240
    uint8_t *jpeg_buf = heap_caps_malloc(jpeg_buf_size, MALLOC_CAP_SPIRAM);
    if (!jpeg_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG copy buffer");
        CTX()->flags.is_web_connected = false;
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        heap_caps_free(jpeg_buf);
        CTX()->flags.is_web_connected = false;
        return res;
    }

    while (true) {
        camera_fb_t *fb = NULL;
        // 等待 app_main 投喂
        if (xQueueReceive(CTX()->q_camera_frame, &fb, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 立即拷贝 JPEG 数据并归还 fb，最小化 fb 占用时间
            size_t jpeg_len = fb->len;
            if (jpeg_len > jpeg_buf_size) jpeg_len = jpeg_buf_size;
            memcpy(jpeg_buf, fb->buf, jpeg_len);
            esp_camera_fb_return(fb);
            fb = NULL;

            // 用拷贝后的数据发送，此时 fb 已归还驱动池
            res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, (ssize_t)strlen(STREAM_BOUNDARY));
            if (res == ESP_OK) {
                size_t h_len = snprintf(part_buf, 64, STREAM_PART, (unsigned)jpeg_len);
                res = httpd_resp_send_chunk(req, part_buf, (ssize_t)h_len);
            }
            if (res == ESP_OK) {
                res = httpd_resp_send_chunk(req, (const char *)jpeg_buf, (ssize_t)jpeg_len);
            }
        }
        if (res != ESP_OK) break;
    }

    heap_caps_free(jpeg_buf);
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

// === 舵机控制 API ===
// POST /api/servo  body: {"x":90,"y":90,"eyelid":90}
static esp_err_t servo_handler(httpd_req_t *req) {
    char buf[128] = {0};
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }

    // 简易 JSON 解析：找 "x": "y": "eyelid": 后面的数字
    int x = CTX()->servo.x, y = CTX()->servo.y, eyelid = CTX()->servo.eyelid;
    char *p;

    p = strstr(buf, "\"x\"");
    if (p) { p += 3; while (*p && *p != ':') p++; if (*p) x = atoi(p + 1); }

    p = strstr(buf, "\"y\"");
    if (p) { p += 3; while (*p && *p != ':') p++; if (*p) y = atoi(p + 1); }

    p = strstr(buf, "\"eyelid\"");
    if (p) { p += 8; while (*p && *p != ':') p++; if (*p) eyelid = atoi(p + 1); }

    servo_set((int16_t)x, (int16_t)y, (int16_t)eyelid);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// === AI 检测结果 API ===
// GET /api/detections → 返回最新检测框 JSON
static esp_err_t detections_handler(httpd_req_t *req) {
    // 紧凑 JSON：{"d":[{"x1":0,"y1":0,"x2":50,"y2":50,"c":0,"s":0.85}],"t":12345}
    char resp[512];
    int off = 0;

    off += snprintf(resp + off, sizeof(resp) - off, "{\"d\":[");

    int count = CTX()->detections.count;
    for (int i = 0; i < count && i < MAX_DETECTIONS; i++) {
        detection_t *d = &CTX()->detections.items[i];
        if (i > 0) off += snprintf(resp + off, sizeof(resp) - off, ",");
        off += snprintf(resp + off, sizeof(resp) - off,
                        "{\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d,\"c\":%d,\"s\":%.2f}",
                        d->x1, d->y1, d->x2, d->y2, d->category, d->score);
    }

    off += snprintf(resp + off, sizeof(resp) - off, "],\"t\":%lld}",
                    CTX()->detections.timestamp);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

esp_err_t start_webserver(void) {
    // === Server 1: 主页 + API（端口 80）===
    // 这些 handler 都是短请求，不会阻塞 httpd 线程
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.stack_size = 8192;
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;

    ESP_LOGI(TAG, "Starting API server on port 80");
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &index_uri);
        httpd_uri_t ota_uri = {
            .uri       = "/update",
            .method    = HTTP_POST,
            .handler   = ota_update_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &ota_uri);
        httpd_uri_t servo_uri = {
            .uri       = "/api/servo",
            .method    = HTTP_POST,
            .handler   = servo_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &servo_uri);
        httpd_uri_t det_uri = {
            .uri       = "/api/detections",
            .method    = HTTP_GET,
            .handler   = detections_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &det_uri);
    } else {
        ESP_LOGE(TAG, "Error starting API server!");
        return ESP_FAIL;
    }

    // === Server 2: MJPEG 推流（端口 81）===
    // stream_handler 是无限循环，会独占 httpd 线程，
    // 所以必须用独立的 httpd 实例，否则会阻塞所有 API 请求
    httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
    stream_config.server_port = 81;
    stream_config.ctrl_port = 32769;  // 控制端口不能和 server 1 冲突
    stream_config.core_id = 0;
    stream_config.stack_size = 8192;
    stream_config.max_uri_handlers = 2;

    httpd_handle_t stream_server = NULL;

    ESP_LOGI(TAG, "Starting stream server on port 81");
    if (httpd_start(&stream_server, &stream_config) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(stream_server, &stream_uri);
    } else {
        ESP_LOGE(TAG, "Error starting stream server!");
        return ESP_FAIL;
    }

    return ESP_OK;
}