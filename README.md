# z-can2pwm

Receives CAN messages and controls four motor outputs using PWM.

## Motor command frame

- Standard 11-bit CAN identifier: `0x100`
- Four-byte frame: one signed two's-complement `int8` speed per motor, in `MOTOR_PWM_PINS` order
- Two-byte single-motor frame: `[channel, speed]`, where channel is `0`-`3` and speed is a signed two's-complement `int8`
- Valid speed range: `-100` to `100`; negative values reverse the motor and zero stops it
- Bitrate: configured by `CAN_BITRATE` in `main/board_config.h` (125 kbit/s, 250 kbit/s, 500 kbit/s, or 1 Mbit/s)
- Safety behavior: all motors stop if no valid command frame arrives for 100 ms
