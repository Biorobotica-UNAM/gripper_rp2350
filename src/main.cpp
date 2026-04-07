#include <cstdio>
#include <cmath>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "w5500.h"
#include "tcp_ip.h"
#include "quadrature_encoder.hpp"
#include "motor_controller.hpp"

// --- Configuration ---
#define ADC_CHANNEL     2u     
#define ADC_GPIO_PIN    28u    
#define ADC_VREF        3.3f   
#define ADC_RESOLUTION  4096.0f

#define EN_MOTOR_PIN    5u
#define PH_MOTOR_PIN    6u
#define ENCODER_CH_A    27u
#define ENCODER_CH_B    7u
#define CPR             48u
#define GEAR_RATIO      26u
#define OUTPUT_CPR      (CPR * GEAR_RATIO)

#define K_P             0.1f
#define K_I             0.02f
#define K_D             0.001f
#define CONTROL_HZ      1000
#define CONTROL_US      (1000000 / CONTROL_HZ)
#define SETPOINT        150.0f

#define CURRENT_THR     2.0f  // Volts

// --- Initialization ---
static void main_core0(void);
static void main_core1(void);

static void setup(void);
static void fsr_adc_init(void);
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
    static uint16_t current;
    static bool dir;
    static bool update_cmd = false;
    static float setpoint = 0.0f;
    uint8_t cmd = STOP;
    status_t motor_status = IDLE;

    while (true) {
        // Gets action from Core 1
        while (multicore_fifo_rvalid()) {
            uint32_t msg = multicore_fifo_pop_blocking();
            cmd          = (uint8_t)msg;
            update_cmd   = true;
        }

        if (cmd == STOP && update_cmd) {
            update_cmd   = false;
            printf("CMD RECEIVED: STOP\n");
            setpoint = 0.0f;
        } else if (cmd == OPEN && update_cmd) {
            update_cmd   = false;
            printf("CMD RECEIVED: OPEN\n");
            setpoint = SETPOINT;
            gpio_put(PH_MOTOR_PIN, true);
        } else if (cmd == CLOSE && update_cmd) {
            update_cmd   = false;
            printf("CMD RECEIVED: CLOSE\n");
            setpoint = SETPOINT;
            gpio_put(PH_MOTOR_PIN, false);
        } else if (cmd == QUERY_STATUS && update_cmd) {
            update_cmd   = false ;
            printf("CMD RECEIVED: QUERY_STATUS\n");
        }

        if (ctrl) {
            ctrl = false;
            // /* Take a single 12-bit sample */
            // raw = adc_read();

            // /* Convert raw value to voltage */
            // float voltage = raw * (ADC_VREF / ADC_RESOLUTION);

            // printf("Raw: %4u  |  Voltage: %.4f V\n", raw, voltage);

            quadrature_encoder_update();
            current_count = quadrature_encoder_get_count();

            absolute_time_t now = get_absolute_time();
            current_speed = get_current_speed(current_count);

            motor.Control(setpoint, current_speed);

            if (fabs(current_speed) > 0 && current < CURRENT_THR) {
                motor_status = MOVING;
            } else if (current > CURRENT_THR) {
                motor_status = STALLED;    
            } else if (fabs(current_speed) == 0 && current < CURRENT_THR) {
                motor_status = IDLE;
            }

            report = (((uint32_t)(((uint16_t)fabs(current_speed)) & 0x0FFF)) << 20) |
                        (((uint32_t)((uint16_t)current & 0x0FFF)) << 8) |
                        ((uint32_t)(motor_status & 0xFF));
            if (!multicore_fifo_wready()) continue;
            multicore_fifo_push_blocking(report);
        }
    }
}

void main_core1(void) {
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

    uint16_t actual_rpm     = 0;
    uint16_t actual_current = 0;
    status_t motor_status   = IDLE;
    uint8_t  pkt[8];

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

        // Send command
        uint8_t  cmd        = pkt[0];
        if (!multicore_fifo_wready()) continue;
        multicore_fifo_push_blocking((uint32_t)cmd);
        tcp_send_response(pkt, actual_rpm, actual_current, motor_status);

        sleep_ms(1);
    }
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