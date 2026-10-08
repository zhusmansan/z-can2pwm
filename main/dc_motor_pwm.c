#include "dc_motor_pwm.h"
#include "board_config.h"

#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "dc_motor_pwm";

#define PWM_FREQ_HZ 50000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY ((1U << PWM_RESOLUTION) - 1U)
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0

typedef struct {
    motor_pwm_pin_config_t pins;
    ledc_channel_t ledc_channel;
    gpio_num_t active_gpio;
    bool reverse;
} dc_motor_pwm_channel_t;

static dc_motor_pwm_channel_t g_channels[DC_MOTOR_PWM_CHANNEL_COUNT];
static bool g_initialized = false;

static gpio_num_t direction_gpio(const dc_motor_pwm_channel_t *motor, bool reverse)
{
    return reverse ? motor->pins.reverse_gpio : motor->pins.forward_gpio;
}

static void set_duty(dc_motor_pwm_channel_t *motor, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(PWM_SPEED_MODE, motor->ledc_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_SPEED_MODE, motor->ledc_channel));
}

static void configure_ledc_channel(dc_motor_pwm_channel_t *motor, gpio_num_t gpio)
{
    ledc_channel_config_t channel_config = {
        .gpio_num = gpio,
        .speed_mode = PWM_SPEED_MODE,
        .channel = motor->ledc_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    motor->active_gpio = gpio;
}

static void set_direction(dc_motor_pwm_channel_t *motor, bool reverse)
{
    if (motor->reverse == reverse) {
        return;
    }

    set_duty(motor, 0);
    ESP_ERROR_CHECK(gpio_reset_pin(motor->active_gpio));
    ESP_ERROR_CHECK(gpio_set_direction(motor->active_gpio, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(motor->active_gpio, 0));
    configure_ledc_channel(motor, direction_gpio(motor, reverse));
    motor->reverse = reverse;
}

void dc_motor_pwm_init(void)
{
    if (g_initialized) {
        return;
    }

    memset(g_channels, 0, sizeof(g_channels));

    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (size_t i = 0; i < (sizeof(MOTOR_PWM_PINS) / sizeof(MOTOR_PWM_PINS[0])); ++i) {
        dc_motor_pwm_channel_t *motor = &g_channels[i];
        motor->pins = MOTOR_PWM_PINS[i];
        motor->ledc_channel = (ledc_channel_t)i;
        motor->active_gpio = motor->pins.forward_gpio;
        configure_ledc_channel(motor, motor->active_gpio);

        gpio_config_t inactive_pin_config = {
            .pin_bit_mask = 1ULL << motor->pins.reverse_gpio,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&inactive_pin_config));
        ESP_ERROR_CHECK(gpio_set_level(motor->pins.reverse_gpio, 0));
    }

    g_initialized = true;
    ESP_LOGI(TAG, "PWM driver initialized for %d channels", DC_MOTOR_PWM_CHANNEL_COUNT);
}

void dc_motor_pwm_set_speed(uint8_t channel, int16_t speed_percent)
{
    if (channel >= DC_MOTOR_PWM_CHANNEL_COUNT) {
        ESP_LOGW(TAG, "Invalid channel %u", channel);
        return;
    }

    if (!g_initialized) {
        dc_motor_pwm_init();
    }
    dc_motor_pwm_channel_t *motor = &g_channels[channel];

    if (speed_percent > 100) {
        speed_percent = 100;
    } else if (speed_percent < -100) {
        speed_percent = -100;
    }

    if (speed_percent == 0) {
        set_duty(motor, 0);
        return;
    }

    bool reverse = speed_percent < 0;
    set_direction(motor, reverse);
    uint32_t duty = ((uint32_t)abs(speed_percent) * PWM_MAX_DUTY + 50U) / 100U;
    set_duty(motor, duty);
}

void dc_motor_pwm_stop_all(void)
{
    for (size_t i = 0; i < DC_MOTOR_PWM_CHANNEL_COUNT; ++i) {
        set_duty(&g_channels[i], 0);
    }
}
