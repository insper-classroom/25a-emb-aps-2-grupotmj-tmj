#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include "Fusion.h"
#include <stdio.h>
#include "hc06.h"      // driver HC-06

#define SAMPLE_PERIOD       (0.01f)       // 10 ms
#define MPU_ADDRESS         0x68
#define I2C_SDA_GPIO        4
#define I2C_SCL_GPIO        5

// UART0 → USB serial
#define UART_ID             uart0
#define UART_BAUD           115200
#define UART_TX_PIN         0
#define UART_RX_PIN         1

// Encoder quadrature
#define ENC_A_PIN           2
#define ENC_B_PIN           3


typedef struct {
    float roll;
    float pitch;
    float yaw;
    bool click;
    int16_t encoder_delta;
} imu_packet_t;

static QueueHandle_t xQueueIMU;
volatile int16_t encoder_count = 0;
static int last_enc_state = 0;

// -----------------------------------------------------------
// callback de IRQ para quadrature rotary encoder
void encoder_gpio_callback(uint gpio, uint32_t events) {
    int a = gpio_get(ENC_A_PIN);
    int b = gpio_get(ENC_B_PIN);
    int state = (a << 1) | b;
    int8_t diff = 0;

    // transições CW
    if ((last_enc_state == 0b00 && state == 0b01) ||
        (last_enc_state == 0b01 && state == 0b11) ||
        (last_enc_state == 0b11 && state == 0b10) ||
        (last_enc_state == 0b10 && state == 0b00)) {
        diff = +1;
    }
    // transições CCW
    else if ((last_enc_state == 0b00 && state == 0b10) ||
             (last_enc_state == 0b10 && state == 0b11) ||
             (last_enc_state == 0b11 && state == 0b01) ||
             (last_enc_state == 0b01 && state == 0b00)) {
        diff = -1;
    }
    encoder_count += diff;
    last_enc_state = state;
}

static void setup_encoder() {
    gpio_set_dir(ENC_A_PIN, GPIO_IN);
    gpio_set_dir(ENC_B_PIN, GPIO_IN);
    gpio_pull_up(ENC_A_PIN);
    gpio_pull_up(ENC_B_PIN);
    last_enc_state = (gpio_get(ENC_A_PIN)<<1) | gpio_get(ENC_B_PIN);

    // registra callback para alterações em A e B
    gpio_set_irq_enabled_with_callback(ENC_A_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true, &encoder_gpio_callback);
    gpio_set_irq_enabled(ENC_B_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true);
}


// -----------------------------------------------------------
// tarefas MPU e UART
static void mpu6050_reset() {
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
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO); gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    imu_packet_t pkt = {0};

    for (;;) {
        mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);

        FusionVector g = {
            .axis.x = gyro_raw[0] / 131.0f,   // cada gyro_raw/131.0f converte LSB→°/s
            .axis.y = gyro_raw[1] / 131.0f,
            .axis.z = gyro_raw[2] / 131.0f
        };
        FusionVector a = {
            .axis.x = accel_raw[0] / 16384.0f,   // cada accel_raw/16384.0f converte LSB→g
            .axis.y = accel_raw[1] / 16384.0f,
            .axis.z = accel_raw[2] / 16384.0f
        };
        FusionAhrsUpdateNoMagnetometer(&ahrs, g, a, SAMPLE_PERIOD);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        static float last_pitch = 0;
        pkt.roll  = e.angle.roll;
        pkt.pitch = e.angle.pitch;
        pkt.yaw   = e.angle.yaw;
        pkt.click = ((pkt.pitch - last_pitch) > 50.0f);
        last_pitch = pkt.pitch;

        pkt.encoder_delta = encoder_count;
        encoder_count = 0;

        xQueueSend(xQueueIMU, &pkt, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void uart_task(void *pvParameters) {
    imu_packet_t pkt;
    char buf[80];

    // inicializa USB
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // inicializa HC-06
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    hc06_init("MeuControle", "1234");

    for (;;) {
        if (xQueueReceive(xQueueIMU, &pkt, portMAX_DELAY) == pdPASS) {
            int len = snprintf(buf, sizeof(buf),
                "%.1f,%.1f,%.1f,%d,%d\n",
                pkt.roll, pkt.pitch, pkt.yaw,
                pkt.click ? 1 : 0,
                pkt.encoder_delta
            );
            uart_write_blocking(UART_ID, (uint8_t*)buf, len);
            uart_write_blocking(HC06_UART_ID,  (uint8_t*)buf, len);
        }
    }
}

int main(void) {
    stdio_init_all();
    xQueueIMU = xQueueCreate(4, sizeof(imu_packet_t));
    setup_encoder();
    xTaskCreate(mpu_task , "MPU6050", 4096, NULL, 2, NULL);
    xTaskCreate(uart_task, "UART"   , 2048, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1);
    return 0;
}