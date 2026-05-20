#include "web_server.h"
#include "context.h"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "servo.h"
#include "tracking.h"
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

static const char *TAG = "web_server";
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Web 推流最大帧率限制（调低可节省带宽和 CPU）
#define STREAM_MAX_FPS 30
#define STREAM_FRAME_INTERVAL_US (1000000 / STREAM_MAX_FPS)

// 推流帧率统计（stream_handler 写，detections_handler 读）
static volatile float s_stream_fps = 0;

// === 日志环形缓冲区 ===
#define LOG_BUF_SIZE 4096
static char s_log_buf[LOG_BUF_SIZE];
static volatile uint32_t s_log_write_pos = 0; // 累计写入字节数（单调递增）

static int log_vprintf_hook(const char *fmt, va_list args) {
  char tmp[256];
  int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
  if (len > 0) {
    int actual = (len < (int)sizeof(tmp)) ? len : (int)sizeof(tmp) - 1;
    for (int i = 0; i < actual; i++) {
      s_log_buf[s_log_write_pos % LOG_BUF_SIZE] = tmp[i];
      s_log_write_pos++;
    }
    // 同时输出到 UART，串口监视器仍然可用
    fwrite(tmp, 1, actual, stdout);
  }
  return len;
}

static esp_err_t ota_update_handler(httpd_req_t *req) {
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
  esp_ota_handle_t update_handle = 0;
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
  if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle) !=
      ESP_OK) {
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

  int64_t last_frame_time = 0;
  int fps_count = 0;
  int64_t fps_last_time = esp_timer_get_time();

  while (true) {
    camera_fb_t *fb = NULL;
    // 等待 app_main 投喂
    if (xQueueReceive(CTX()->q_camera_frame, &fb, pdMS_TO_TICKS(100)) ==
        pdTRUE) {
      // 帧率限制：如果距离上一帧时间不够，跳过这帧
      int64_t now = esp_timer_get_time();
      if (now - last_frame_time < STREAM_FRAME_INTERVAL_US) {
        esp_camera_fb_return(fb);
        continue;
      }
      last_frame_time = now;

      // 立即拷贝 JPEG 数据并归还 fb，最小化 fb 占用时间
      size_t jpeg_len = fb->len;
      if (jpeg_len > jpeg_buf_size)
        jpeg_len = jpeg_buf_size;
      memcpy(jpeg_buf, fb->buf, jpeg_len);
      esp_camera_fb_return(fb);
      fb = NULL;

      // 用拷贝后的数据发送，此时 fb 已归还驱动池
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                  (ssize_t)strlen(STREAM_BOUNDARY));
      if (res == ESP_OK) {
        size_t h_len = snprintf(part_buf, 64, STREAM_PART, (unsigned)jpeg_len);
        res = httpd_resp_send_chunk(req, part_buf, (ssize_t)h_len);
      }
      if (res == ESP_OK) {
        res = httpd_resp_send_chunk(req, (const char *)jpeg_buf,
                                    (ssize_t)jpeg_len);
      }

      // 统计推流帧率
      if (res == ESP_OK) {
        fps_count++;
        now = esp_timer_get_time();
        if (now - fps_last_time >= 1000000) {
          s_stream_fps =
              fps_count * 1000000.0f / (float)(now - fps_last_time);
          fps_count = 0;
          fps_last_time = now;
        }
      }
    }
    if (res != ESP_OK)
      break;
  }

  s_stream_fps = 0;
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
  // 追踪开启时禁止手动控制
  if (tracking_is_enabled()) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":false,\"reason\":\"tracking active\"}");
  }

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
  if (p) {
    p += 3;
    while (*p && *p != ':')
      p++;
    if (*p)
      x = atoi(p + 1);
  }

  p = strstr(buf, "\"y\"");
  if (p) {
    p += 3;
    while (*p && *p != ':')
      p++;
    if (*p)
      y = atoi(p + 1);
  }

  p = strstr(buf, "\"eyelid\"");
  if (p) {
    p += 8;
    while (*p && *p != ':')
      p++;
    if (*p)
      eyelid = atoi(p + 1);
  }

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
    if (i > 0)
      off += snprintf(resp + off, sizeof(resp) - off, ",");
    off += snprintf(
        resp + off, sizeof(resp) - off,
        "{\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d,\"c\":%d,\"s\":%.2f}", d->x1,
        d->y1, d->x2, d->y2, d->category, d->score);
  }

  off += snprintf(resp + off, sizeof(resp) - off,
                  "],\"t\":%lld,\"fps\":%.1f}",
                  CTX()->detections.timestamp, s_stream_fps);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, resp);
}

// === 抓拍 API ===
// GET /api/snapshot → 返回当前 JPEG 帧，触发浏览器下载
static esp_err_t snapshot_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Snapshot: camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition",
                     "attachment; filename=snapshot.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

// === AI 开关 API ===
// POST /api/ai  body: {"enabled":true}
static esp_err_t ai_toggle_handler(httpd_req_t *req) {
  char buf[64] = {0};
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    return ESP_FAIL;
  }

  bool enabled = (strstr(buf, "true") != NULL);
  CTX()->flags.ai_enabled = enabled;

  // 关闭 AI 时同时关闭追踪，并清空检测结果
  if (!enabled) {
    tracking_set_enabled(false);
    CTX()->detections.count = 0;
  }

  ESP_LOGI(TAG, "AI inference %s", enabled ? "ON" : "OFF");
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, enabled ? "{\"enabled\":true}"
                                         : "{\"enabled\":false}");
}

// === 追踪开关 API ===
// POST /api/tracking  body: {"enabled":true}
static esp_err_t tracking_toggle_handler(httpd_req_t *req) {
  char buf[64] = {0};
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    return ESP_FAIL;
  }

  bool enabled = (strstr(buf, "true") != NULL);
  tracking_set_enabled(enabled);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, enabled ? "{\"enabled\":true}"
                                         : "{\"enabled\":false}");
}

// === PID 参数读取/设置 API ===
// GET  /api/pid_params → 返回当前参数
// POST /api/pid_params → 设置新参数  body: {"kp":0.1,"ki":0.05,"kd":0.0,"max_inc":10}
static esp_err_t pid_params_get_handler(httpd_req_t *req) {
  tracking_params_t p = tracking_get_params();
  char resp[128];
  snprintf(resp, sizeof(resp),
           "{\"kp\":%.4f,\"ki\":%.4f,\"kd\":%.4f,\"max_inc\":%.1f}",
           p.kp, p.ki, p.kd, p.max_increment);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, resp);
}

static esp_err_t pid_params_set_handler(httpd_req_t *req) {
  char buf[128] = {0};
  int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
  if (len <= 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
    return ESP_FAIL;
  }

  tracking_params_t p = tracking_get_params(); // 保留未修改的值
  char *ptr;

  ptr = strstr(buf, "\"kp\"");
  if (ptr) { ptr = strchr(ptr, ':'); if (ptr) p.kp = strtof(ptr + 1, NULL); }

  ptr = strstr(buf, "\"ki\"");
  if (ptr) { ptr = strchr(ptr, ':'); if (ptr) p.ki = strtof(ptr + 1, NULL); }

  ptr = strstr(buf, "\"kd\"");
  if (ptr) { ptr = strchr(ptr, ':'); if (ptr) p.kd = strtof(ptr + 1, NULL); }

  ptr = strstr(buf, "\"max_inc\"");
  if (ptr) { ptr = strchr(ptr, ':'); if (ptr) p.max_increment = strtof(ptr + 1, NULL); }

  tracking_set_params(&p);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, "{\"ok\":true}");
}

// === 日志 API ===
// GET /api/logs?pos=N → 返回 pos 之后的新日志内容
static esp_err_t logs_handler(httpd_req_t *req) {
  char query[32] = {0};
  uint32_t client_pos = 0;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char val[16];
    if (httpd_query_key_value(query, "pos", val, sizeof(val)) == ESP_OK) {
      client_pos = (uint32_t)strtoul(val, NULL, 10);
    }
  }

  uint32_t current_pos = s_log_write_pos;
  uint32_t available = current_pos - client_pos;

  // 客户端太落后，截断到缓冲区大小
  if (available > LOG_BUF_SIZE) {
    client_pos = current_pos - LOG_BUF_SIZE;
    available = LOG_BUF_SIZE;
  }

  // 构建响应: {"pos":N,"d":"...log text..."}
  // 先发 header
  char header[48];
  snprintf(header, sizeof(header), "{\"pos\":%lu,\"d\":\"", (unsigned long)current_pos);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send_chunk(req, header, strlen(header));

  // 发送日志内容（需要 JSON 转义）
  if (available > 0) {
    char escaped[512];
    int esc_idx = 0;
    for (uint32_t i = 0; i < available; i++) {
      char c = s_log_buf[(client_pos + i) % LOG_BUF_SIZE];
      if (c == '"') { escaped[esc_idx++] = '\\'; escaped[esc_idx++] = '"'; }
      else if (c == '\\') { escaped[esc_idx++] = '\\'; escaped[esc_idx++] = '\\'; }
      else if (c == '\n') { escaped[esc_idx++] = '\\'; escaped[esc_idx++] = 'n'; }
      else if (c == '\r') { /* skip */ }
      else if (c >= 0x20) { escaped[esc_idx++] = c; }

      // 刷新缓冲区
      if (esc_idx > (int)sizeof(escaped) - 4) {
        httpd_resp_send_chunk(req, escaped, esc_idx);
        esc_idx = 0;
      }
    }
    if (esc_idx > 0) {
      httpd_resp_send_chunk(req, escaped, esc_idx);
    }
  }

  httpd_resp_send_chunk(req, "\"}", 2);
  httpd_resp_send_chunk(req, NULL, 0); // 结束 chunked
  return ESP_OK;
}

esp_err_t start_webserver(void) {
  // === Server 1: 主页 + API（端口 80）===
  // 这些 handler 都是短请求，不会阻塞 httpd 线程
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.core_id = 0;
  config.stack_size = 8192;
  config.max_uri_handlers = 12;

  httpd_handle_t server = NULL;

  ESP_LOGI(TAG, "Starting API server on port 80");
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t index_uri = {.uri = "/",
                             .method = HTTP_GET,
                             .handler = index_handler,
                             .user_ctx = NULL};
    httpd_register_uri_handler(server, &index_uri);
    httpd_uri_t ota_uri = {.uri = "/update",
                           .method = HTTP_POST,
                           .handler = ota_update_handler,
                           .user_ctx = NULL};
    httpd_register_uri_handler(server, &ota_uri);
    httpd_uri_t servo_uri = {.uri = "/api/servo",
                             .method = HTTP_POST,
                             .handler = servo_handler,
                             .user_ctx = NULL};
    httpd_register_uri_handler(server, &servo_uri);
    httpd_uri_t det_uri = {.uri = "/api/detections",
                           .method = HTTP_GET,
                           .handler = detections_handler,
                           .user_ctx = NULL};
    httpd_register_uri_handler(server, &det_uri);
    httpd_uri_t snap_uri = {.uri = "/api/snapshot",
                            .method = HTTP_GET,
                            .handler = snapshot_handler,
                            .user_ctx = NULL};
    httpd_register_uri_handler(server, &snap_uri);
    httpd_uri_t ai_uri = {.uri = "/api/ai",
                          .method = HTTP_POST,
                          .handler = ai_toggle_handler,
                          .user_ctx = NULL};
    httpd_register_uri_handler(server, &ai_uri);
    httpd_uri_t tracking_uri = {.uri = "/api/tracking",
                                .method = HTTP_POST,
                                .handler = tracking_toggle_handler,
                                .user_ctx = NULL};
    httpd_register_uri_handler(server, &tracking_uri);
    httpd_uri_t pid_get_uri = {.uri = "/api/pid_params",
                               .method = HTTP_GET,
                               .handler = pid_params_get_handler,
                               .user_ctx = NULL};
    httpd_register_uri_handler(server, &pid_get_uri);
    httpd_uri_t pid_set_uri = {.uri = "/api/pid_params",
                               .method = HTTP_POST,
                               .handler = pid_params_set_handler,
                               .user_ctx = NULL};
    httpd_register_uri_handler(server, &pid_set_uri);
    httpd_uri_t logs_uri = {.uri = "/api/logs",
                            .method = HTTP_GET,
                            .handler = logs_handler,
                            .user_ctx = NULL};
    httpd_register_uri_handler(server, &logs_uri);
  } else {
    ESP_LOGE(TAG, "Error starting API server!");
    return ESP_FAIL;
  }

  // === Server 2: MJPEG 推流（端口 81）===
  // stream_handler 是无限循环，会独占 httpd 线程，
  // 所以必须用独立的 httpd 实例，否则会阻塞所有 API 请求
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = 32769; // 控制端口不能和 server 1 冲突
  stream_config.core_id = 0;
  stream_config.stack_size = 8192;
  stream_config.max_uri_handlers = 2;

  httpd_handle_t stream_server = NULL;

  ESP_LOGI(TAG, "Starting stream server on port 81");
  if (httpd_start(&stream_server, &stream_config) == ESP_OK) {
    httpd_uri_t stream_uri = {.uri = "/stream",
                              .method = HTTP_GET,
                              .handler = stream_handler,
                              .user_ctx = NULL};
    httpd_register_uri_handler(stream_server, &stream_uri);
  } else {
    ESP_LOGE(TAG, "Error starting stream server!");
    return ESP_FAIL;
  }

  // 启用日志捕获钩子
  esp_log_set_vprintf(log_vprintf_hook);
  ESP_LOGI(TAG, "Log capture enabled");

  return ESP_OK;
}