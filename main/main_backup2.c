// main.c
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include "Fusion.h"
#include <stdio.h>
#include "hc06.h"
#include "hardware/pwm.h"

#define SAMPLE_PERIOD       (0.01f)       // 10 ms
#define MPU_ADDRESS         0x68
#define I2C_SDA_GPIO        4
#define I2C_SCL_GPIO        5

#define UART_ID             uart0
#define UART_BAUD           115200
#define UART_TX_PIN         0
#define UART_RX_PIN         1

#define ENC_A_PIN           2
#define ENC_B_PIN           3

#define LED1_R              11
#define LED1_G              12
#define LED1_B              13
#define LED2_R              14
#define LED2_G              15
#define LED2_B              16

#define PWM_WRAP            1000
#define PWM_CLKDIV          64.0f

#define BTN_AIM_PIN         17   // “mirar” → left mouse button
#define BTN_FIRE_PIN        18   // “atirar” → right mouse button

volatile int16_t encoder_count = 0;
static int last_enc_state = 0;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    bool click;
    int16_t encoder_delta;
    bool aim;
    bool fire;
} imu_packet_t;

static QueueHandle_t xQueueIMU;

// -------------------------------------------------------------------
static void setup_leds(void) {
    const int leds[] = {
        LED1_R, LED1_G, LED1_B,
        LED2_R, LED2_G, LED2_B
    };
    for (size_t i = 0; i < sizeof(leds)/sizeof(leds[0]); ++i) {
        int pin = leds[i];
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        pwm_set_clkdiv(slice, PWM_CLKDIV);
        pwm_set_wrap(slice, PWM_WRAP);
        pwm_set_chan_level(slice,
            pwm_gpio_to_channel(pin),
            0
        );
        pwm_set_enabled(slice, true);
    }
}

static void set_rgb1(uint16_t r, uint16_t g, uint16_t b) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED1_R),
                      pwm_gpio_to_channel(LED1_R), r);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED1_G),
                      pwm_gpio_to_channel(LED1_G), g);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED1_B),
                      pwm_gpio_to_channel(LED1_B), b);
}

static void set_rgb2(uint16_t r, uint16_t g, uint16_t b) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED2_R),
                      pwm_gpio_to_channel(LED2_R), r);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED2_G),
                      pwm_gpio_to_channel(LED2_G), g);
    pwm_set_chan_level(pwm_gpio_to_slice_num(LED2_B),
                      pwm_gpio_to_channel(LED2_B), b);
}

static void led_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t base_delay = pdMS_TO_TICKS(1500 - 100); // 1900 ms
    const TickType_t flash_delay = pdMS_TO_TICKS(100);       // 100 ms

    for (;;) {
        // cores base: LED1 = vermelho, LED2 = verde
        set_rgb1(PWM_WRAP, 0, 0);
        set_rgb2(0, PWM_WRAP, 0);
        // espera até o próximo flash
        vTaskDelay(base_delay);

        // flash branco rápido
        set_rgb1(0, 0, PWM_WRAP);
        set_rgb2(0, 0, PWM_WRAP);
        vTaskDelay(flash_delay);
        // ao retornar ao topo do for(), as cores base reaparecem
    }
}

// -------------------------------------------------------------------
// callback de IRQ para encoder
void encoder_gpio_callback(uint gpio, uint32_t events) {
    (void)events;
    int a = gpio_get(ENC_A_PIN);
    int b = gpio_get(ENC_B_PIN);
    int state = (a << 1) | b;
    int8_t diff = 0;
    if ((last_enc_state == 0b00 && state == 0b01) ||
        (last_enc_state == 0b01 && state == 0b11) ||
        (last_enc_state == 0b11 && state == 0b10) ||
        (last_enc_state == 0b10 && state == 0b00)) {
        diff = +1;
    } else if ((last_enc_state == 0b00 && state == 0b10) ||
               (last_enc_state == 0b10 && state == 0b11) ||
               (last_enc_state == 0b11 && state == 0b01) ||
               (last_enc_state == 0b01 && state == 0b00)) {
        diff = -1;
    }
    encoder_count += diff;
    last_enc_state = state;
}

static void setup_io(void) {
    // Botões: apenas input + pull-up
    gpio_set_dir(BTN_AIM_PIN,  GPIO_IN);
    gpio_pull_up(BTN_AIM_PIN);
    gpio_set_dir(BTN_FIRE_PIN, GPIO_IN);
    gpio_pull_up(BTN_FIRE_PIN);

    // Encoder quadrature
    gpio_set_dir(ENC_A_PIN, GPIO_IN);
    gpio_set_dir(ENC_B_PIN, GPIO_IN);
    gpio_pull_up(ENC_A_PIN);
    gpio_pull_up(ENC_B_PIN);
    last_enc_state = (gpio_get(ENC_A_PIN)<<1) | gpio_get(ENC_B_PIN);
    gpio_set_irq_enabled_with_callback(
        ENC_A_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_gpio_callback
    );
    gpio_set_irq_enabled(
        ENC_B_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );
}

// -------------------------------------------------------------------
static void mpu6050_reset(void) {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6], reg;
    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer , 6, false);
    for(int i=0;i<3;i++) accel[i] = (buffer[2*i]<<8)|buffer[2*i+1];
    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer , 6, false);
    for(int i=0;i<3;i++) gyro[i] = (buffer[2*i]<<8)|buffer[2*i+1];
    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer , 2, false);
    *temp = (buffer[0]<<8)|buffer[1];
}

static void mpu_task(void *pvParameters) {
    (void)pvParameters;
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    imu_packet_t pkt = {0};

    float last_pitch = 0;

    for (;;) {
        // Leitura IMU
        mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);
        FusionVector g = {
            .axis.x = gyro_raw[0] / 131.0f,
            .axis.y = gyro_raw[1] / 131.0f,
            .axis.z = gyro_raw[2] / 131.0f
        };
        FusionVector a = {
            .axis.x = accel_raw[0] / 16384.0f,
            .axis.y = accel_raw[1] / 16384.0f,
            .axis.z = accel_raw[2] / 16384.0f
        };
        FusionAhrsUpdateNoMagnetometer(&ahrs, g, a, SAMPLE_PERIOD);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Preenche o pacote
        pkt.roll  = e.angle.roll;
        pkt.pitch = -e.angle.pitch;
        pkt.yaw   = e.angle.yaw;
        pkt.click = ((pkt.pitch - last_pitch) > 1.0f);
        last_pitch = pkt.pitch;

        pkt.encoder_delta = encoder_count;
        encoder_count = 0;

        // ** Polling dos botões a cada ciclo **
        pkt.aim  = (gpio_get(BTN_AIM_PIN)  == 0);
        pkt.fire = (gpio_get(BTN_FIRE_PIN) == 0);

        xQueueSend(xQueueIMU, &pkt, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void uart_task(void *pvParameters) {
    (void)pvParameters;
    imu_packet_t pkt;
    char buf[80];
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    hc06_init("MeuControle", "1234");

    for (;;) {
        if (xQueueReceive(xQueueIMU, &pkt, portMAX_DELAY) == pdPASS) {
            int len = snprintf(buf, sizeof(buf),
                "%.1f,%.1f,%.1f,%d,%d,%d,%d\n",
                pkt.roll,
                pkt.pitch,
                pkt.yaw,
                pkt.click  ? 1 : 0,
                pkt.encoder_delta,
                pkt.aim    ? 1 : 0,
                pkt.fire   ? 1 : 0
            );
            uart_write_blocking(UART_ID, (uint8_t*)buf, len);
            uart_write_blocking(HC06_UART_ID, (uint8_t*)buf, len);
        }
    }
}

int main(void) {
    stdio_init_all();
    setup_leds();
    xQueueIMU = xQueueCreate(4, sizeof(imu_packet_t));

    setup_io();

    xTaskCreate(mpu_task , "MPU6050", 4096, NULL, 2, NULL);
    xTaskCreate(uart_task, "UART"   , 2048, NULL, 1, NULL);
    xTaskCreate(led_task,  "LED"    , 1024, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) { tight_loop_contents(); }
    return 0;
}