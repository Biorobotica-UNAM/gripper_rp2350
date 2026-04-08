#include "motor_controller.hpp"

MotorController::MotorController(float k_p, float k_i, float k_d, uint8_t pwm_pin) {
	this->kp 	  = k_p;
	this->ki 	  = k_i;
	this->kd 	  = k_d;
	this->pwm_pin = pwm_pin;

	gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    this->slice_num = pwm_gpio_to_slice_num(pwm_pin);
    this->pwm_chan = pwm_gpio_to_channel(pwm_pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLK_DIV);
    pwm_config_set_wrap(&config, PWM_WRAP);
    
    pwm_init(this->slice_num, &config, true);
    pwm_set_chan_level(this->slice_num, this->pwm_chan, 0);
}

void MotorController::Control(float setpoint, float current_speed) {
	static uint16_t pwm_level;
	static float profile_setpoint;

	if (setpoint == 0) {
		pwm_set_chan_level(this->slice_num, this->pwm_chan, 0);
	} else {
		// profile_setpoint = MotorController::GenerateProfile(setpoint);
		// this->err[2] = profile_setpoint - fabs(current_speed);
		this->err[2] = setpoint - fabs(current_speed);

		this->u[1] = u[0] + this->kp * (this->err[2] - this->err[1]) 
						+ this->ki * (this->err[2] + this->err[1]) / 2.0f 
						+ this->kd * (this->err[2] - 2 * this->err[1] + this->err[0]);

		if (this->u[1] > 1.0f) {
			this->u[1] = 1.0f;
		} else if (this->u[1] < 0.0f) {
			this->u[1] = 0.0f;
		}

		// pwm_level = (uint16_t)(this->u[1] * (PWM_WRAP + 1));
		pwm_level = (uint16_t)(this->u[1] * (PWM_WRAP - 50));
		pwm_set_chan_level(this->slice_num, this->pwm_chan, pwm_level);

		this->u[0] = this->u[1];
		this->err[0] = this->err[1];
		this->err[1] = this->err[2];
	}
}

// TODO
float MotorController::GenerateProfile(float setpoint) {
	static float speed = setpoint;
	return speed;
}