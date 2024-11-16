#ifndef ULTRASONIC_ENCODER_H
#define ULTRASONIC_ENCODER_H

void wheel_encoder_init();
void ultrasonic_init();
bool ultrasonic_timer_callback(struct repeating_timer *t);

extern volatile bool blocked;
extern volatile float left_speed;
extern volatile float right_speed;
#endif