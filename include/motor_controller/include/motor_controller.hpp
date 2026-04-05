#pragma once
#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// PWM freq = 20kHz
#define PWM_WRAP        7499u
#define PWM_CLK_DIV     1.0f

class MotorController {
public:
	MotorController(float k_p, float k_i, float k_d, uint8_t pwm_pin);
	void MotorControl(float setpoint, float current_speed);

private:
	float kp;
	float ki;
	float kd;
	float err[3] = {0, 0, 0};
	float u[2] = {0, 0};
	uint8_t pwm_pin;
	uint8_t slice_num;
	uint8_t pwm_chan;
};