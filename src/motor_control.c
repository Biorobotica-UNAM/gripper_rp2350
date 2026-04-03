#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

/* ── Pin definitions ────────────────────────────────────────────── */
#define MOTOR_PWM_PIN   5u   // GPIO3  – PWM output
#define MOTOR_DIR_PIN   6u   // GPIO4  – direction output

/* ── PWM configuration ──────────────────────────────────────────── */
/*
 * RP2350 system clock default: 150 MHz
 * Target PWM frequency       : ~20 kHz  (above audible range)
 *
 * wrap  = (sys_clk / (divider * f_pwm)) - 1
 *       = (150_000_000 / (1 * 20_000)) - 1  = 7499
 *
 * duty  = (speed_percent / 100) * (wrap + 1)
 */
#define PWM_WRAP        7499u          // gives 20 kHz @ 150 MHz, div=1
#define PWM_CLK_DIV     1.0f           // integer+fractional divider

/* ── Direction constants ─────────────────────────────────────────── */
#define DIR_FORWARD     true
#define DIR_BACKWARD    false

/* ── Helper: initialise PWM slice for MOTOR_PWM_PIN ─────────────── */
static uint motor_pwm_slice;
static uint motor_pwm_chan;

static void motor_pwm_init(void)
{
    gpio_set_function(MOTOR_PWM_PIN, GPIO_FUNC_PWM);

    motor_pwm_slice = pwm_gpio_to_slice_num(MOTOR_PWM_PIN);
    motor_pwm_chan  = pwm_gpio_to_channel(MOTOR_PWM_PIN);

    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_CLK_DIV);
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    pwm_init(motor_pwm_slice, &cfg, true);   // start the slice immediately
    pwm_set_chan_level(motor_pwm_slice, motor_pwm_chan, 0); // 0 % duty
}

/* ── Helper: initialise direction GPIO ──────────────────────────── */
static void motor_dir_init(void)
{
    gpio_init(MOTOR_DIR_PIN);
    gpio_set_dir(MOTOR_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR_DIR_PIN, DIR_FORWARD);    // default direction
}

/* ── Public API ──────────────────────────────────────────────────── */

/**
 * motor_set_speed_pct()
 *   speed_pct : 0–100  (0 = stop, 100 = full speed)
 */
void motor_set_speed_pct(uint8_t speed_pct)
{
    if (speed_pct > 100) speed_pct = 100;

    uint32_t level = ((uint32_t)speed_pct * (PWM_WRAP + 1u)) / 100u;
    pwm_set_chan_level(motor_pwm_slice, motor_pwm_chan, (uint16_t)level);
}

/**
 * motor_set_direction()
 *   forward : true  → GPIO4 HIGH
 *             false → GPIO4 LOW
 *
 * IMPORTANT: always set speed to 0 before reversing direction
 * to protect the driver from shoot-through / back-EMF spikes.
 */
void motor_set_direction(bool forward)
{
    gpio_put(MOTOR_DIR_PIN, forward ? 1 : 0);
}

/**
 * motor_stop()
 *   Immediately drop PWM duty to 0.
 */
void motor_stop(void)
{
    motor_set_speed_pct(0);
}

/* ── Entry point ─────────────────────────────────────────────────── */
int main(void)
{
    stdio_init_all();   // optional – useful for debug prints over USB/UART

    sleep_ms(2000);

    motor_pwm_init();
    motor_dir_init();

    printf("Motor control ready\n");
    printf("PWM pin : GPIO%u  (slice %u, chan %u)\n",
           MOTOR_PWM_PIN, motor_pwm_slice, motor_pwm_chan);
    printf("DIR pin : GPIO%u\n", MOTOR_DIR_PIN);

    /* ── Demo sequence ──────────────────────────────────────────── */

    // 1) Ramp up forward
    motor_set_direction(DIR_FORWARD);
    printf("Forward – ramping up...\n");
    for (int s = 0; s <= 100; s += 5) {
        motor_set_speed_pct((uint8_t)s);
        sleep_ms(100);
    }

    sleep_ms(2000);

    // 2) Ramp down, switch direction, ramp up backward
    printf("Stopping before direction change...\n");
    for (int s = 100; s >= 0; s -= 5) {
        motor_set_speed_pct((uint8_t)s);
        sleep_ms(50);
    }
    sleep_ms(200);  // brief pause – let motor fully stop

    motor_set_direction(DIR_BACKWARD);
    printf("Backward – ramping up...\n");
    for (int s = 0; s <= 75; s += 5) {
        motor_set_speed_pct((uint8_t)s);
        sleep_ms(100);
    }

    sleep_ms(2000);

    // 3) Full stop
    printf("Stopping.\n");
    motor_stop();

    while (true) {
        tight_loop_contents();   // idle
    }

    return 0;
}