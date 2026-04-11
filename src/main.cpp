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
#define ADC_RESOLUTION  4096.0f
#define ADC_VREF        3.3f   

#define FSR0_CHANNEL    0u                      // RIGHT FINGER
#define FSR0_GPIO_PIN   (26 + FSR0_CHANNEL)

#define FSR1_CHANNEL    1u                      // LEFT FINGER
#define FSR1_GPIO_PIN   (26 + FSR1_CHANNEL)

// --- TEST IN PROTO ---
// #define CS_CHANNEL      1u
// #define CS_GPIO_PIN     (26 + CS_CHANNEL)

#define CS_CHANNEL      2u
#define CS_GPIO_PIN     (26 + CS_CHANNEL)

#define EN_MOTOR_PIN    0u
#define PH_MOTOR_PIN    7u
#define ENCODER_CH_A    5u
#define ENCODER_CH_B    6u

// --- TEST IN PROTO ---
// #define EN_MOTOR_PIN    5u
// #define PH_MOTOR_PIN    6u
// #define ENCODER_CH_A    27u
// #define ENCODER_CH_B    7u

#define CPR             48u
#define GEAR_RATIO      26u
#define OUTPUT_CPR      (CPR * GEAR_RATIO)

#define K_P             0.1f
#define K_I             0.02f
#define K_D             0.001f
#define CONTROL_HZ      1000
#define CONTROL_US      (1000000 / CONTROL_HZ)
#define SETPOINT        150.0f

#define CURRENT_THR     3.3f  // amps
#define FORCE_THR       1100  

// --- Initialization ---
static void main_core0(void);
static void main_core1(void);

static void setup(void);
static void inst_adc_init(void);
static void digital_gpio_init();
static void encoders_gpio_init(void);

// --- Synchronization ---
static mutex_t      report_mutex;
static semaphore_t  report_sem;
static motor_report_t shared_report;

void sync_init(void);

// --- Motor Control ---
repeating_timer_t timer;
volatile bool ctrl = false;
int32_t last_count = 0;
MotorController motor(K_P, K_I, K_D, EN_MOTOR_PIN);
static absolute_time_t last_time = get_absolute_time();

static float get_current_speed(int32_t current_count);
static bool control_callback(repeating_timer_t *rt);

int main(void) {
    setup();

    multicore_launch_core1(main_core1);

    main_core0();

    return 0;
}

void main_core0(void) {
    static uint32_t report;
    static int32_t current_count;
    static float current_speed;
    static uint16_t raw;
    static uint16_t force;
    static float current;
    static bool dir;
    static float setpoint = 0.0f;
    static uint16_t adc_values[3];
    uint8_t cmd = STOP;
    status_t motor_status = IDLE;

    adc_run(true);
    adc_fifo_drain();
    adc_select_input(FSR0_CHANNEL);

    while (true) {
        // Gets action from Core 1
        while (multicore_fifo_rvalid()) {
            uint32_t msg = multicore_fifo_pop_blocking();
            cmd          = (uint8_t)msg;
        }

        if (adc_fifo_get_level() >= 3) {
            for(uint8_t i = 0; i < 3; i++) {
                uint16_t raw = adc_fifo_get();

                bool adc_err = raw & 0x8000;
                if (!adc_err) adc_values[i] = raw & 0x0FFF;
            }

            // Drain any extra samples to avoid misalignment on next read
            while (!adc_fifo_is_empty()) adc_fifo_get();

            // 1.13 [V/A] (Check DRV8874 datasheet)
            current = 0.885 * adc_values[CS_CHANNEL] * ADC_VREF / ADC_RESOLUTION;
            // printf("FSR0: %u | FSR1: %u | Current: %.2f\n", adc_values[FSR0_CHANNEL], adc_values[FSR1_CHANNEL], current);
            printf("Current: %.2f [A]\n", current);
        }

        switch (motor_status) {
            case IDLE: 
                switch (cmd) {
                    case OPEN:
                        setpoint = SETPOINT;
                        gpio_put(PH_MOTOR_PIN, true);
                        printf("Command received: OPEN\n");
                        motor_status = OPENING;
                        break;
                    case CLOSE:
                        setpoint = SETPOINT;
                        gpio_put(PH_MOTOR_PIN, false);
                        printf("Command received: CLOSE\n");
                        motor_status = CLOSING;
                        break;
                }
                break;

            case OPENING:
                // Force stop conditions
                if (cmd == STOP) {
                    setpoint     = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = IDLE;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    motor_status = IDLE;
                    break;
                }

                // Autonomous transitions
                // if (current > CURRENT_THR) {
                //     motor_status = STALLED;
                // } else if (fabs(current_speed) == 0 && current < CURRENT_THR) {
                //     setpoint = 0.0f;
                //     motor_status = IDLE;
                // }
                break;

            case CLOSING:
                // Force stop conditions
                if (cmd == STOP) {
                    setpoint     = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = IDLE;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    motor_status = IDLE;
                    break;
                }

                // Autonomous transitions
                // if (current > CURRENT_THR) {
                //     motor_status = STALLED;
                // } else if (fabs(current_speed) == 0 && current < CURRENT_THR) {
                //     setpoint = 0.0f;
                //     motor_status = IDLE;
                // }

                if (adc_values[FSR0_CHANNEL] + adc_values[FSR1_CHANNEL] / 2 > FORCE_THR) {
                    motor_status = REACHED;
                }

                break;

            case STALLED:
                setpoint     = 0.0f;

                mutex_enter_blocking(&report_mutex);
                shared_report.rpm     = 0;
                shared_report.current = (uint16_t)current;
                shared_report.status  = STALLED;
                mutex_exit(&report_mutex);

                sem_release(&report_sem);

                motor_status = IDLE;
                break;

            case REACHED:
                setpoint = 0.0f;

                mutex_enter_blocking(&report_mutex);
                shared_report.rpm     = (uint16_t)fabs(current_speed);
                shared_report.current = (uint16_t)current;
                shared_report.status  = motor_status;
                mutex_exit(&report_mutex);

                sem_release(&report_sem);       // unblocks Core 1

                motor_status = IDLE;
                break;
        }

        if (ctrl) {
            ctrl = false;

            quadrature_encoder_update();
            current_count = quadrature_encoder_get_count();

            absolute_time_t now = get_absolute_time();
            current_speed = get_current_speed(current_count);

            motor.Control(setpoint, current_speed);
        }

        cmd = 0;
    }
}

void main_core1(void) {
    uint16_t actual_rpm     = 0;
    uint16_t actual_current = 0;
    status_t motor_status   = IDLE;
    uint8_t  pkt[8];

    w5500_init_spi();
    sleep_ms(200);

    if (!tcp_init()) {
        printf("TCP init: FAILED\n");
        while (true) tight_loop_contents();
    }

    if (!tcp_open()) {
        printf("Open TCP: FAILED\n");
        while (true) tight_loop_contents();
    }

    while (true) {
        // Get status updates from Core 0
        while (multicore_fifo_rvalid()) {
            uint32_t msg   = multicore_fifo_pop_blocking();
            actual_rpm     = (uint16_t)(msg >> 20 & 0x0FFF);
            actual_current = (uint16_t)(msg >> 8 & 0x0FFF);
            motor_status   = (status_t)(msg & 0x00FF);
        }

        // Check connection state
        uint8_t sr = w5500_read_sock_byte(TCP_SOCKET, Sn_SR);

        if (sr == SOCK_CLOSE_WAIT || sr == SOCK_CLOSED) {
            // Client disconnected — re-arm listener
            tcp_reconnect();
            sleep_ms(10);
            continue;
        }

        if (sr != SOCK_ESTABLISHED) {
            // Still waiting for a client to connect — nothing to do
            sleep_ms(5);
            continue;
        }

        // Check how many bytes are available in RX buffer
        uint8_t rsr[2];
        w5500_dma_read(Sn_RX_RSR, BSB_SOCK_REG(TCP_SOCKET), rsr, 2);
        uint16_t rx_size = ((uint16_t)rsr[0] << 8) | rsr[1];

        // Wait until a full 8-byte command has arrived
        if (rx_size < 8) {
            sleep_ms(1);
            continue;
        }

        // Read exactly 8 bytes
        uint8_t rd_ptr[2];
        w5500_dma_read(Sn_RX_RD, BSB_SOCK_REG(TCP_SOCKET), rd_ptr, 2);
        uint16_t rx_rd = ((uint16_t)rd_ptr[0] << 8) | rd_ptr[1];

        w5500_dma_read(rx_rd, BSB_SOCK_RX(TCP_SOCKET), pkt, 8);

        rx_rd += 8;
        uint8_t new_rd[2] = { rx_rd >> 8, rx_rd & 0xFF };
        _w5500_write_blocking(Sn_RX_RD, BSB_SOCK_REG(TCP_SOCKET), new_rd, 2);
        w5500_write_sock_byte(TCP_SOCKET, Sn_CR, Sn_CR_RECV);

        uint8_t  cmd        = pkt[0];
        if (!multicore_fifo_wready()) continue;
        multicore_fifo_push_blocking((uint32_t)cmd);

        if (cmd == OPEN || cmd == CLOSE || cmd == STOP) {
            sem_acquire_blocking(&report_sem);   // blocks here until Core 0 signals

            mutex_enter_blocking(&report_mutex);
            actual_rpm     = shared_report.rpm;
            actual_current = shared_report.current;
            motor_status   = shared_report.status;
            mutex_exit(&report_mutex);
        }

        tcp_send_response(pkt, actual_rpm, actual_current, motor_status);

        cmd = 0;
    }
}

void setup(void) {
    stdio_init_all();

    /* Give the host a moment to attach a serial terminal */
    sleep_ms(2000);

    add_repeating_timer_us(-CONTROL_US, control_callback, NULL, &timer);

    inst_adc_init();
    digital_gpio_init();
    quadrature_encoder_init(pio0, 0, ENCODER_CH_A, ENCODER_CH_B, 1.0f);

    sync_init();

    printf("XIAO RP2350 correctly configured!\n");
}

void inst_adc_init(void) {
    adc_init();

    adc_gpio_init(FSR0_GPIO_PIN);
    adc_gpio_init(FSR1_GPIO_PIN);
    adc_gpio_init(CS_GPIO_PIN);

    adc_select_input(FSR0_CHANNEL);

    adc_set_round_robin((1 << CS_CHANNEL) | (1 << FSR1_CHANNEL) | (1 << FSR0_CHANNEL));

    adc_fifo_setup(
        true,
        false,
        1,
        true,
        false
    );

    adc_set_clkdiv(96000.0f);
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

void sync_init(void) {
    mutex_init(&report_mutex);
    sem_init(&report_sem, 0, 1);   // starts at 0 (not ready), max 1
}