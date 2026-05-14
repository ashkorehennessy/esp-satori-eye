#include "servo.h"
#include "context.h"
#include "esp_log.h"
#include "driver/ledc.h"

static const char *TAG = "Servo";

// LEDC 配置
// 摄像头占用了 LEDC_TIMER_0 + LEDC_CHANNEL_0，舵机使用 TIMER_1 + CHANNEL_1/2/3
#define SERVO_LEDC_TIMER        LEDC_TIMER_1
#define SERVO_LEDC_MODE         LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_RESOLUTION   LEDC_TIMER_14_BIT   // 14-bit: 16384 ticks/period
#define SERVO_LEDC_MAX_DUTY     ((1 << 14) - 1)      // 16383

#define SERVO_CH_X              LEDC_CHANNEL_1
#define SERVO_CH_Y              LEDC_CHANNEL_2
#define SERVO_CH_EYELID         LEDC_CHANNEL_3

// 角度限幅
static int16_t clamp_angle(int16_t angle) {
    if (angle < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
    return angle;
}

// 角度 → LEDC duty
// pulse_us = 500 + angle * (2400 - 500) / 180
// duty = pulse_us * 16384 / 20000
static uint32_t angle_to_duty(int16_t angle) {
    uint32_t pulse_us = SERVO_MIN_PULSE_US +
        (uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / SERVO_MAX_ANGLE;
    // 20000us = 50Hz 周期
    return pulse_us * (SERVO_LEDC_MAX_DUTY + 1) / 20000;
}

static void servo_channel_init(ledc_channel_t channel, int gpio_num) {
    ledc_channel_config_t ch_conf = {
        .gpio_num   = gpio_num,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = channel,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}

void servo_init(void) {
    // 配置 LEDC 定时器：50Hz, 14-bit 分辨率
    ledc_timer_config_t timer_conf = {
        .speed_mode      = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_RESOLUTION,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // 配置 3 个通道
    servo_channel_init(SERVO_CH_X,      SERVO_PIN_X);
    servo_channel_init(SERVO_CH_Y,      SERVO_PIN_Y);
    servo_channel_init(SERVO_CH_EYELID, SERVO_PIN_EYELID);

    // 立即转到中位 (90°)
    servo_set(90, 90, 90);

    ESP_LOGI(TAG, "Servo initialized — X:GPIO%d Y:GPIO%d Eyelid:GPIO%d → 90°",
             SERVO_PIN_X, SERVO_PIN_Y, SERVO_PIN_EYELID);
}

void servo_set(int16_t x, int16_t y, int16_t eyelid) {
    x = clamp_angle(x);
    y = clamp_angle(y);
    eyelid = clamp_angle(eyelid);

    CTX()->servo.x = x;
    CTX()->servo.y = y;
    CTX()->servo.eyelid = eyelid;

    ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH_X,      angle_to_duty(x));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH_X);

    ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH_Y,      angle_to_duty(y));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH_Y);

    ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH_EYELID,  angle_to_duty(eyelid));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH_EYELID);

    ESP_LOGI(TAG, "Set → X:%d° Y:%d° Eyelid:%d°", x, y, eyelid);
}
