#include "servo.h"
#include "context.h"
#include "esp_log.h"

static const char *TAG = "Servo";

// 角度限幅
static int16_t clamp_angle(int16_t angle) {
    if (angle < SERVO_MIN_ANGLE) return SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) return SERVO_MAX_ANGLE;
    return angle;
}

void servo_init(void) {
    // TODO: 初始化 LEDC PWM 通道（3路）
    // ledc_timer_config(...)
    // ledc_channel_config(...) x3
    ESP_LOGI(TAG, "Servo initialized (stub) — X:%d Y:%d Eyelid:%d",
             CTX()->servo.x, CTX()->servo.y, CTX()->servo.eyelid);
}

void servo_set(int16_t x, int16_t y, int16_t eyelid) {
    x = clamp_angle(x);
    y = clamp_angle(y);
    eyelid = clamp_angle(eyelid);

    CTX()->servo.x = x;
    CTX()->servo.y = y;
    CTX()->servo.eyelid = eyelid;

    // TODO: 实际 PWM 输出
    // uint32_t duty_x = angle_to_duty(x);
    // ledc_set_duty(...);
    // ledc_update_duty(...);

    ESP_LOGI(TAG, "Set → X:%d Y:%d Eyelid:%d", x, y, eyelid);
}
