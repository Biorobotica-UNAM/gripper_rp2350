#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "quadrature_encoder.hpp"
#include "motor_controller.hpp"

// --- Configuration ---
#define ADC_CHANNEL     2u     
#define ADC_GPIO_PIN    28u    
#define ADC_VREF        3.3f   
#define ADC_RESOLUTION  4096.0f
#define SAMPLE_DELAY_MS 1u
#define EN_MOTOR_PIN    5u
#define PH_MOTOR_PIN    6u
#define ENCODER_CH_A    27u
#define ENCODER_CH_B    7u
#define CPR             48u
#define GEAR_RATIO      26u
#define OUTPUT_CPR     (CPR * GEAR_RATIO)
#define K_P             0.3f
#define K_I             0.05f
#define K_D             0.001f
#define SPEED_SETPOINT  200.0f
#define CONTROL_HZ      1000
#define CONTROL_US      (1000000 / CONTROL_HZ)

// --- Initialization ---
void setup(void);
void fsr_adc_init(void);
void digital_gpio_init();
void encoders_gpio_init(void);

// --- Motor Control ---
repeating_timer_t timer;
volatile bool ctrl = false;
absolute_time_t last_time = get_absolute_time();
int32_t last_count = 0;
MotorController motor(K_P, K_I, K_D, EN_MOTOR_PIN);

float get_current_speed(int32_t current_count);
bool control_callback(repeating_timer_t *rt);

int main(void) {
    setup();

    while (true) {
        static int32_t current_count;
        static float current_speed;
        static uint16_t raw;

        /* Take a single 12-bit sample */
        raw = adc_read();

        /* Convert raw value to voltage */
        float voltage = raw * (ADC_VREF / ADC_RESOLUTION);

        printf("Raw: %4u  |  Voltage: %.4f V\n", raw, voltage);

        absolute_time_t now = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(last_time, now);

        printf("Control flag: %d\n", ctrl);

        if (ctrl) {
            ctrl = false;

            quadrature_encoder_update();
            current_count = quadrature_encoder_get_count();
            current_speed = get_current_speed(current_count);

            motor.MotorControl(SPEED_SETPOINT, current_speed);

            printf("Count: %ld | RPM: %.2f\n",
                   current_count,
                   std::fabs(current_speed));
        }
        sleep_ms(SAMPLE_DELAY_MS);
    }

    return 0;
}

void setup(void) {
    stdio_init_all();

    /* Give the host a moment to attach a serial terminal */
    sleep_ms(2000);

    add_repeating_timer_us(-CONTROL_US, control_callback, NULL, &timer);

    fsr_adc_init();
    digital_gpio_init();
    quadrature_encoder_init(pio0, 0, ENCODER_CH_A, ENCODER_CH_B, 1.0f);

    printf("XIAO RP2350 correctly configured!\n");
}

void fsr_adc_init(void) {
    adc_init();

    #if ADC_CHANNEL < 4
        adc_gpio_init(ADC_GPIO_PIN);
    #endif

    adc_select_input(ADC_CHANNEL);          // Select the active channel
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