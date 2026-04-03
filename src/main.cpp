#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

/* ── Configuration ──────────────────────────────────────────────── */
#define ADC_CHANNEL     2u           // ADC channel to sample (0-3 for GPIO, 4 for temp)
#define ADC_GPIO_PIN    28u          // Corresponding GPIO pin (26 + channel for ch 0-3)
#define ADC_VREF        3.3f        // Reference voltage in volts
#define ADC_RESOLUTION  4096.0f    // 12-bit ADC → 2^12 steps
#define SAMPLE_DELAY_MS 500u         // Delay between samples in milliseconds
#define EN_MOTOR_PIN    5u
#define PH_MOTOR_PIN    6u
#define ENCODER_CH_A    7u
#define ENCODER_CH_B    0u
#define CPR             48u
#define GEAR_RATIO      26u
#define OUTPUT_CPR     (CPR * GEAR_RATIO)
#define PWM_WRAP        7500u        // PWM freq = 20kHz
#define PWM_CLK_DIV     1.0f
/* ────────────────────────────────────────────────────────────────── */

volatile bool speed_update = false;
volatile int32_t pulse_count = 0;
volatile int32_t last_count  = 0;
static absolute_time_t  last_time;

static uint8_t count = 0;
static bool dir = false;
static uint slice_num;
static uint pwm_chan;

// --- Initialization ---
void setup(void);
void fsr_adc_init(void);
void motor_pwm_init(void);
void digital_gpio_init();
void encoders_gpio_init(void);

// --- ISR ---
void encoder_isr(uint gpio, uint32_t events);

int main(void) {
    setup();

    pwm_set_chan_level(slice_num, pwm_chan, 7450);
    last_time = get_absolute_time();

    while (true) {
        /* Take a single 12-bit sample */
        uint16_t raw = adc_read();

        /* Convert raw value to voltage */
        float voltage = raw * (ADC_VREF / ADC_RESOLUTION);

        printf("Raw: %4u  |  Voltage: %.4f V\n", raw, voltage);

        if(speed_update) {
            speed_update = false;

            // Snapshot atomically (disable IRQ briefly)
            uint32_t irq_status = save_and_disable_interrupts();
            int32_t current_count = pulse_count;
            restore_interrupts(irq_status);

            absolute_time_t now       = get_absolute_time();
            int32_t         delta     = current_count - last_count;
            float           dt_sec    = absolute_time_diff_us(last_time, now) / 1e6f;

            // Counts per second → revolutions per second → RPM
            float counts_per_sec = delta / dt_sec;
            float rps            = counts_per_sec / (OUTPUT_CPR * 4.0f); // x4 = quadrature
            float rpm            = rps * 60.0f;

            last_count = current_count;
            last_time  = now;

            printf("Count: %ld | RPM: %.2f | Dir: %s\n",
                   current_count,
                   std::fabs(rpm),
                   (delta >= 0) ? "CW" : "CCW");
        }

        if(count >= 40) {
            count = 0;
            dir = !dir;
            gpio_put(PH_MOTOR_PIN, dir);
            printf("Cambio de sentido: %d\n", dir);
        } else {
            count++;
        };

        sleep_ms(SAMPLE_DELAY_MS);
    }

    return 0;
}

void setup(void) {
    /* Initialize stdio (UART or USB, depending on CMakeLists.txt) */
    stdio_init_all();

    /* Give the host a moment to attach a serial terminal */
    sleep_ms(2000);

    fsr_adc_init();
    motor_pwm_init();
    digital_gpio_init();
    encoders_gpio_init();

    printf("XIAO RP2350 correctly configured!\n");
}

void fsr_adc_init(void) {
    /* ── ADC initialisation ──────────────────────────────────────── */
    adc_init();                             // Power up and reset the ADC block

    /* Only call adc_gpio_init() for external channels (0-3).
       Channel 4 is the internal temperature sensor — no GPIO needed. */
    #if ADC_CHANNEL < 4
        adc_gpio_init(ADC_GPIO_PIN);            // Set GPIO to ADC input (hi-Z, no pull)
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
    gpio_put(PH_MOTOR_PIN, dir);
}

void encoders_gpio_init(void) {
    // Encoders config
    gpio_init(ENCODER_CH_A);
    gpio_init(ENCODER_CH_B);
    gpio_set_dir(ENCODER_CH_A, GPIO_IN);
    gpio_set_dir(ENCODER_CH_B, GPIO_IN);
    gpio_pull_up(ENCODER_CH_A);
    gpio_pull_up(ENCODER_CH_B);

    // IRQ config
    gpio_set_irq_enabled_with_callback(ENCODER_CH_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    gpio_set_irq_enabled(ENCODER_CH_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void encoder_isr(uint gpio, uint32_t events) {
    gpio_acknowledge_irq(gpio, events);

    bool a = gpio_get(ENCODER_CH_A);
    bool b = gpio_get(ENCODER_CH_B);

    if (gpio == ENCODER_CH_A) {
        // Rising edge on A
        pulse_count += (a == b) ? -1 : +1;
    } else {
        // Rising or falling edge on B
        pulse_count += (a == b) ? +1 : -1;
    }

    speed_update = true;
}