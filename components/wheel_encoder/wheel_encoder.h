#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H

void wheel_encoder_init();

extern volatile float left_speed;
extern volatile float right_speed;
extern volatile uint32_t pulses_left;
extern volatile uint32_t pulses_right;

#endif