#ifndef DC_MOTOR_PWM_H
#define DC_MOTOR_PWM_H

#include <stdint.h>
#include <stdbool.h>


void dc_motor_pwm_init(void);
void dc_motor_pwm_set_speed(uint8_t channel, int16_t speed_percent);
void dc_motor_pwm_stop_all(void);

#endif
