#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void motor_init();
void move_forward(float duty_cycle_A, float duty_cycle_B);
void move_backward(float duty_cycle_A, float duty_cycle_B);
void stop_motors();
void turn_right(float duty_cycle_A, float duty_cycle_B);
#endif