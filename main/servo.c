#include "servo.h"
#include "context.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "Servo";

// LEDC 配置
// 摄像头占用了 LEDC_TIMER_0 + LEDC_CHANNEL_0，舵机使用 TIMER_1 + CHANNEL_1/2/3
#define SERVO_LEDC_TIMER LEDC_TIMER_1
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_RESOLUTION LEDC_TIMER_14_BIT // 14-bit: 16384 ticks/period
#define SERVO_LEDC_MAX_DUTY ((1 << 14) - 1)     // 16383

#define SERVO_CH_1 LEDC_CHANNEL_1
#define SERVO_CH_2 LEDC_CHANNEL_2
#define SERVO_CH_3 LEDC_CHANNEL_3

// 角度限幅
static int16_t clamp_angle(int16_t angle) {
  if (angle < SERVO_MIN_ANGLE)
    return SERVO_MIN_ANGLE;
  if (angle > SERVO_MAX_ANGLE)
    return SERVO_MAX_ANGLE;
  return angle;
}

// 角度 → LEDC duty
// pulse_us = 500 + angle * (2400 - 500) / 180
// duty = pulse_us * 16384 / 20000
static uint32_t angle_to_duty(int16_t angle) {
  uint32_t pulse_us =
      SERVO_MIN_PULSE_US + (uint32_t)angle *
                               (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) /
                               SERVO_MAX_ANGLE;
  // 20000us = 50Hz 周期
  return pulse_us * (SERVO_LEDC_MAX_DUTY + 1) / 20000;
}

static void servo_channel_init(ledc_channel_t channel, int gpio_num) {
  ledc_channel_config_t ch_conf = {
      .gpio_num = gpio_num,
      .speed_mode = SERVO_LEDC_MODE,
      .channel = channel,
      .timer_sel = SERVO_LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
      .intr_type = LEDC_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
}

void servo_init(void) {
  // 配置 LEDC 定时器：50Hz, 14-bit 分辨率
  ledc_timer_config_t timer_conf = {
      .speed_mode = SERVO_LEDC_MODE,
      .duty_resolution = SERVO_LEDC_RESOLUTION,
      .timer_num = SERVO_LEDC_TIMER,
      .freq_hz = SERVO_FREQ_HZ,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  // 配置 3 个通道
  servo_channel_init(SERVO_CH_1, SERVO_PIN_1);
  servo_channel_init(SERVO_CH_2, SERVO_PIN_2);
  servo_channel_init(SERVO_CH_3, SERVO_PIN_3);

  // 初始化到默认位置：X=100, Y=90, eyelid=70(大约七成开)
  servo_set(100, 90, 70);

  ESP_LOGI(TAG, "Servo initialized — S1:GPIO%d S2:GPIO%d S3:GPIO%d",
           SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3);
}

// === 底层接口：直接控制三路舵机 ===
void servo_set_raw(int16_t s1, int16_t s2, int16_t s3) {
  s1 = clamp_angle(s1);
  s2 = clamp_angle(s2);
  s3 = clamp_angle(s3);

  ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH_1, angle_to_duty(s1));
  ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH_1);

  ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH_2, angle_to_duty(s2));
  ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH_2);

  ledc_set_duty(SERVO_LEDC_MODE, SERVO_CH_3, angle_to_duty(s3));
  ledc_update_duty(SERVO_LEDC_MODE, SERVO_CH_3);
}

// === 高级接口：逻辑控制（自动处理 Y-眼皮联动）===
void servo_set(int16_t x, int16_t y, int16_t eyelid) {
  // 限幅逻辑参数
  if (eyelid < 0) eyelid = 0;
  if (eyelid > 100) eyelid = 100;

  // 物理通道映射
  int16_t s1 = x;    // servo1 = X 轴直通
  int16_t s2 = y;    // servo2 = Y 轴直通

  // servo3 = Y-眼皮联动公式
  // servo3 = RATIO * servo2 + OFFSET - SCALE * eyelid
  float s3f = EYELID_Y_RATIO * (float)s2 + EYELID_Y_OFFSET - EYELID_SCALE * (float)eyelid;
  int16_t s3 = (int16_t)s3f;
  // 联动输出安全限幅
  if (s3 < 35) s3 = 35;
  if (s3 > 130) s3 = 130;

  // 保存逻辑状态到 context
  CTX()->servo.x = x;
  CTX()->servo.y = y;
  CTX()->servo.eyelid = eyelid;

  // 下发到硬件
  servo_set_raw(s1, s2, s3);
}
