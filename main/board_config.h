#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "driver/gpio.h"

#define CAN_RX_GPIO     0
#define CAN_TX_GPIO     1
#define CAN_GND_GPIO    2
#define CAN_VCC_GPIO    3
#define CAN_QUEUE_DEPTH        1000
#define CAN_BITRATE            1000000

typedef struct {
    gpio_num_t forward_gpio;
    gpio_num_t reverse_gpio;
} motor_pwm_pin_config_t;

static const motor_pwm_pin_config_t MOTOR_PWM_PINS[] = {
    {GPIO_NUM_5, GPIO_NUM_6},
    {GPIO_NUM_7, GPIO_NUM_8},
    {GPIO_NUM_9, GPIO_NUM_10},
    {GPIO_NUM_20, GPIO_NUM_21},
};

#define DC_MOTOR_PWM_CHANNEL_COUNT sizeof(MOTOR_PWM_PINS) / sizeof(MOTOR_PWM_PINS[0])

#endif
