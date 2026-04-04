#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "quadrature_encoder.hpp"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

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
#define PWM_WRAP        7499u                   // PWM freq = 20kHz
#define PWM_CLK_DIV     1.0f
#define SPEED_SETPOINT  200.0f

// --- Motor PWM ---
static uint slice_num;
static uint pwm_chan;

// --- Motor Speed ---
absolute_time_t last_time = get_absolute_time();
int32_t last_count = 0;

// --- Initialization ---
void setup(void);
void fsr_adc_init(void);
void motor_pwm_init(void);
void digital_gpio_init();
void encoders_gpio_init(void);

// --- Motor control ---
float get_current_speed(int32_t current_count);

int main(void) {
    setup();

    pwm_set_chan_level(slice_num, pwm_chan, 3750);

    while (true) {
        quadrature_encoder_update();

        /* Take a single 12-bit sample */
        uint16_t raw = adc_read();

        /* Convert raw value to voltage */
        float voltage = raw * (ADC_VREF / ADC_RESOLUTION);

        printf("Raw: %4u  |  Voltage: %.4f V\n", raw, voltage);

        absolute_time_t now = get_absolute_time();
        int64_t elapsed_us = absolute_time_diff_us(last_time, now);

        if (elapsed_us >= 100000) {
            int32_t current_count = quadrature_encoder_get_count();

            printf("Count: %ld | RPM: %.2f\n",
                   current_count,
                   std::fabs(get_current_speed(current_count)));
        }
        sleep_ms(SAMPLE_DELAY_MS);
    }

    return 0;
}

void setup(void) {
    stdio_init_all();

    /* Give the host a moment to attach a serial terminal */
    sleep_ms(2000);

    fsr_adc_init();
    motor_pwm_init();
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

void motor_pwm_init(void) {
    // PWM config
    gpio_set_function(EN_MOTOR_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(EN_MOTOR_PIN);
    pwm_chan = pwm_gpio_to_channel(EN_MOTOR_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLK_DIV);
    pwm_config_set_wrap(&config, PWM_WRAP);
    
    pwm_init(slice_num, &config, true);
    pwm_set_chan_level(slice_num, pwm_chan, 0);
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