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
#include "protocol.h"
#include "quadrature_encoder.hpp"
#include "motor_controller.hpp"

// --- Configuration ---
#define ADC_RESOLUTION  4096.0f
#define ADC_VREF        3.3f   

#define FSR0_CHANNEL    0u                      // RIGHT FINGER
#define FSR0_GPIO_PIN   (26 + FSR0_CHANNEL)

#define FSR1_CHANNEL    1u                      // LEFT FINGER
#define FSR1_GPIO_PIN   (26 + FSR1_CHANNEL)

#define CS_CHANNEL      2u
#define CS_GPIO_PIN     (26 + CS_CHANNEL)

#define EN_MOTOR_PIN    0u
#define PH_MOTOR_PIN    7u
#define ENCODER_CH_A    5u
#define ENCODER_CH_B    6u

#define CPR             48
#define GEAR_RATIO      26
#define OUTPUT_CPR      (CPR * GEAR_RATIO)
#define OPEN_COUNT      (4 * CPR * GEAR_RATIO)
#define STOP_TIME_MS    1600

#define K_P             0.1f
#define K_I             0.02f
#define K_D             0.001f
#define CONTROL_HZ      1000
#define CONTROL_US      (1000000 / CONTROL_HZ)
#define SETPOINT        150.0f

#define CURRENT_THR     3.3f  // amps
#define FORCE_THR       1500  

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
volatile bool stop = false;
bool stop_timer_armed = false;
alarm_id_t stop_alarm_id;
int32_t last_count = 0;
MotorController motor(K_P, K_I, K_D, EN_MOTOR_PIN);
static absolute_time_t last_time = get_absolute_time();

static float get_current_speed(int32_t current_count);
static bool control_callback(repeating_timer_t *rt);
static int64_t stop_cb(alarm_id_t id, void *user_data);
static inline bool contact_detected(uint16_t *adc);
static inline bool close_contact_detected(uint16_t *adc);

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
    static bool open_dir = true;
    static bool close_dir = false;
    static bool init_entry = true;
    static bool close = false;     // OPEN: false, CLOSE: true
    static float setpoint = 0.0f;
    static uint16_t adc_values[3];
    uint8_t cmd = 0;
    status_t motor_status = INIT;

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
            adc_fifo_drain();

            // 1.13 [V/A] (Check DRV8874 datasheet)
            current = 0.885 * adc_values[CS_CHANNEL] * ADC_VREF / ADC_RESOLUTION;
            printf("FSR0: %u | FSR1: %u | Current: %.2f\n", adc_values[FSR0_CHANNEL], adc_values[FSR1_CHANNEL], current);
            // printf("Current: %.2f [A]\n", current);
        }

        switch (motor_status) {
            case INIT:
                // printf("SM_STATE: INIT\n");

                setpoint = SETPOINT;
                gpio_put(PH_MOTOR_PIN, close_dir);

                if (init_entry) {
                    if (stop_timer_armed) {
                        cancel_alarm(stop_alarm_id);
                    }

                    stop_alarm_id = add_alarm_in_ms(STOP_TIME_MS, stop_cb, NULL, false);
                    stop_timer_armed = true;
                    init_entry = false;
                }

                if (stop) {
                    stop = false;

                    if (stop_timer_armed) {
                        cancel_alarm(stop_alarm_id);
                        stop_timer_armed = false;
                    }

                    stop_alarm_id = add_alarm_in_ms(STOP_TIME_MS, stop_cb, NULL, false);
                    stop_timer_armed = true;
                    quadrature_encoder_clean_count();
                    close = true;
                    motor_status = OPENING;
                }
                break;

            case OPENING:
                // printf("SM_STATE: OPENING\n");
                
                setpoint = SETPOINT;
                gpio_put(PH_MOTOR_PIN, open_dir);

                // Force stop conditions
                quadrature_encoder_update();
                printf("Opening count: %d\n", quadrature_encoder_get_count());

                // Open conditions
                if (quadrature_encoder_get_count() >= (int)OPEN_COUNT) {
                    setpoint = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.pos_cnt = quadrature_encoder_get_count();
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = REACHED;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    close = false;
                    motor_status = IDLE;
                    break;
                } else if (stop) {
                    stop = false;
                    setpoint = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.pos_cnt = quadrature_encoder_get_count();
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = IDLE;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    close = false;
                    motor_status = IDLE;
                    break;
                }

                // Force stop command
                if (cmd == STOP) {
                    setpoint     = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.pos_cnt = quadrature_encoder_get_count();
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = IDLE;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    close = false;
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
                // printf("SM_STATE: CLOSING\n");
                
                setpoint = SETPOINT;
                gpio_put(PH_MOTOR_PIN, close_dir);

                // Force stop command
                if (cmd == STOP) {
                    setpoint     = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.pos_cnt = quadrature_encoder_get_count();
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = REACHED;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    close = true;
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

                // Close conditions
                if (contact_detected(adc_values)) {
                    setpoint     = 0.0f;

                    printf("Contact detected. FSR0: %ld || FSR1: %ld\n", adc_values[FSR0_CHANNEL], adc_values[FSR1_CHANNEL]);
                    
                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.pos_cnt = quadrature_encoder_get_count();
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = REACHED;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    close = true;
                    motor_status = IDLE;
                } else if (stop) {
                    stop = false;
                    setpoint = 0.0f;

                    mutex_enter_blocking(&report_mutex);
                    shared_report.rpm     = 0;
                    shared_report.pos_cnt = quadrature_encoder_get_count();
                    shared_report.current = (uint16_t)current;
                    shared_report.status  = IDLE;
                    mutex_exit(&report_mutex);

                    sem_release(&report_sem);

                    close = true;
                    motor_status = IDLE;
                    break;
                }

                break;

            case IDLE: 
                // printf("SM_STATE: IDLE\n");
                
                switch (cmd) {
                    case OPEN:
                        printf("Command received: OPEN\n");

                        stop = false;

                        if (close) motor_status = OPENING;
                        else {
                            mutex_enter_blocking(&report_mutex);
                            shared_report.rpm     = 0;
                            shared_report.pos_cnt = quadrature_encoder_get_count();
                            shared_report.current = (uint16_t)current;
                            shared_report.status  = IDLE;
                            mutex_exit(&report_mutex);

                            sem_release(&report_sem);
                            break;
                        }

                        if (stop_timer_armed) {
                            cancel_alarm(stop_alarm_id);
                        }

                        stop_alarm_id = add_alarm_in_ms(STOP_TIME_MS, stop_cb, NULL, false);
                        stop_timer_armed = true;

                        break;

                    case CLOSE:
                        printf("Command received: CLOSE\n");

                        stop = false;

                        if (!close) motor_status = CLOSING;
                        else {
                            mutex_enter_blocking(&report_mutex);
                            shared_report.rpm     = 0;
                            shared_report.pos_cnt = quadrature_encoder_get_count();
                            shared_report.current = (uint16_t)current;
                            shared_report.status  = IDLE;
                            mutex_exit(&report_mutex);

                            sem_release(&report_sem);
                            
                            break;
                        }

                        if (stop_timer_armed) {
                            cancel_alarm(stop_alarm_id);
                        }

                        stop_alarm_id = add_alarm_in_ms(STOP_TIME_MS, stop_cb, NULL, false);
                        stop_timer_armed = true;

                        break;
                }
                break;
        }

        if (ctrl) {
            ctrl = false;

            quadrature_encoder_update();
            current_count = quadrature_encoder_get_count();

            absolute_time_t now = get_absolute_time();
            current_speed = get_current_speed(current_count);

            // printf("Current count: %d\n", current_count);

            motor.Control(setpoint, current_speed);
        }

        cmd = 0;
    }
}

void main_core1(void) {
    motor_report_t res = {
        .rpm     = 0,
        .current = 0,
        .pos_cnt = 0,
        .status  = IDLE
    };

    protocol_msg_t msg;
    uint8_t        cmd;

    w5500_init_spi();
    sleep_ms(200);

    if (!tcp_init()) {
        printf("TCP init: FAILED\n");
        while (true) tight_loop_contents();
    }

    if (!tcp_open()) {
        printf("Create socket: FAILED\n");
        while (true) tight_loop_contents();
    }

    while (true) {
        // Check connection state
        if (!is_connected()) {
            sleep_ms(5);
            continue;
        }

        // Wait until the cmd message is received
        if (!protocol_read(&msg)) {
            sleep_ms(5);
            continue;
        }

        cmd = msg.payload[0];

        if (!multicore_fifo_wready()) continue;

        while (sem_try_acquire(&report_sem));

        multicore_fifo_push_blocking((uint32_t)cmd);

        if (cmd == OPEN || cmd == CLOSE || cmd == STOP) {
            sem_acquire_blocking(&report_sem);   // blocks here until Core 0 signals

            mutex_enter_blocking(&report_mutex);
            res.rpm     = shared_report.rpm;
            res.current = shared_report.current;
            res.pos_cnt = shared_report.pos_cnt;
            res.status  = shared_report.status;
            mutex_exit(&report_mutex);
        }

        protocol_send_response(&res);

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

int64_t stop_cb(alarm_id_t id, void *user_data) {
    stop = true;
    return 0;
}

void sync_init(void) {
    mutex_init(&report_mutex);
    sem_init(&report_sem, 0, 1);   // starts at 0 (not ready), max 1
}

bool contact_detected(uint16_t *adc) {
    return (adc[FSR0_CHANNEL] + adc[FSR1_CHANNEL]) / 2 > FORCE_THR;
}

bool close_contact_detected(uint16_t *adc) {
    return (adc[FSR0_CHANNEL] + adc[FSR1_CHANNEL]) / 2 > 300;
}