#include <cstdio>
#include <cmath>
#include <iostream>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/mutex.h"
#include "pico/sem.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "w5500.h"
#include "tcp_ip.h"
#include "quadrature_encoder.hpp"
#include "motor_controller.hpp"

// --- Configuration ---
#define EN_MOTOR_PIN    0u
#define PH_MOTOR_PIN    7u
#define ENCODER_CH_A    5u
#define ENCODER_CH_B    6u

#define CPR             48
#define GEAR_RATIO      26
#define OUTPUT_CPR      (CPR * GEAR_RATIO)
#define OPEN_COUNT      (4 * CPR * GEAR_RATIO)
#define STOP_TIME_MS    2000

#define K_P             0.1f
#define K_I             0.02f
#define K_D             0.001f
#define CONTROL_HZ      1000
#define CONTROL_US      (1000000 / CONTROL_HZ)
#define SETPOINT        150.0f

// --- Initialization ---
static void setup(void);
static void digital_gpio_init();
static void encoders_gpio_init(void);

// --- Motor Control ---
repeating_timer_t timer;
volatile bool ctrl = false;
int32_t last_count = 0;
MotorController motor(K_P, K_I, K_D, EN_MOTOR_PIN);
static absolute_time_t last_time = get_absolute_time();

static float get_current_speed(int32_t current_count);
static bool control_callback(repeating_timer_t *rt);

int main(void) {
    static int32_t current_count;
    static float current_speed;
    static float current;
    static float setpoint = SETPOINT;

    setup();

    while (true) {
        if (ctrl) {
            ctrl = false;

            quadrature_encoder_update();
            current_count = quadrature_encoder_get_count();

            absolute_time_t now = get_absolute_time();
            current_speed = get_current_speed(current_count);

            printf("Current count: %d\n", current_count);

            motor.Control(setpoint, current_speed);
        }
    }
}

void setup(void) {
    stdio_init_all();

    /* Give the host a moment to attach a serial terminal */
    sleep_ms(2000);

    add_repeating_timer_us(-CONTROL_US, control_callback, NULL, &timer);

    digital_gpio_init();
    quadrature_encoder_init(pio0, 0, ENCODER_CH_A, ENCODER_CH_B, 1.0f);

    printf("XIAO RP2350 correctly configured!\n");
}

void digital_gpio_init(void) {
    // GPIO config
    gpio_init(PH_MOTOR_PIN);
    gpio_set_dir(PH_MOTOR_PIN, GPIO_OUT);
    gpio_put(PH_MOTOR_PIN, false);
}

float get_current_speed(int32_t current_count) {
    absolute_time_t now      = get_absolute_time();
    int32_t         delta    = current_count - last_count;
    float           dt_sec   = absolute_time_diff_us(last_time, now) / 1e6f;

    float counts_per_sec = delta / dt_sec;
    float rps            = counts_per_sec / OUTPUT_CPR;
    float rpm            = rps * 60.0f;

    last_count = current_count;
    last_time  = now;

    return rpm;
}

bool control_callback(repeating_timer_t *rt) {
    ctrl = true;
    return ctrl;
}