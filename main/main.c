#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define ADC_CHANNEL_X   0
#define ADC_CHANNEL_Y   1

#define UART_ID         uart0
#define BAUD_RATE       115200
#define UART_TX_PIN     0
#define UART_RX_PIN     1

// Definição dos pinos dos botões
#define BUTTON_A_PIN    14   // Botão que enviará 'A'
#define BUTTON_D_PIN    15   // Botão que enviará 'D'

#define EOP             0xFF

#define FILTER_SIZE     5
#define DEADZONE        50

// Tipos de mensagem
typedef enum {
    TYPE_JOYSTICK,
    TYPE_BUTTON
} message_type_t;

// Estrutura unificada de mensagem
typedef struct {
    message_type_t type;
    union {
        struct {
            int16_t x;
            int16_t y;
        } joystick;
        struct {
            char key;
        } button;
    } data;
} message_t;

QueueHandle_t xQueueMessage;

/* Aplica média móvel a uma sequência de leituras */
static int applyMovingAverage(int newValue, int *buffer, int *index, int *sum) {
    *sum -= buffer[*index];
    buffer[*index] = newValue;
    *sum += newValue;
    *index = (*index + 1) % FILTER_SIZE;
    return (*sum) / FILTER_SIZE;
}

/* Centraliza, escala e aplica a zona morta ao valor do ADC */
static int scaleValue(int adcValue) {
    int centered = adcValue - 2048;
    float scale = 255.0f / 2047.0f;
    int scaled = (int)(centered * scale);
    if (abs(scaled) < DEADZONE)
        return 0;
    return scaled;
}

/*
 * Tarefa de aquisição do ADC (joystick):
 * Lê os dois canais, aplica os filtros e envia uma mensagem do tipo JOYSTICK.
 */
void vTaskADC(void *p) {
    int buffer_x[FILTER_SIZE] = {0}, index_x = 0, sum_x = 0;
    int buffer_y[FILTER_SIZE] = {0}, index_y = 0, sum_y = 0;
    message_t msg;
    
    while(1) {
        // Leitura do eixo X
        adc_select_input(ADC_CHANNEL_X);
        (void)adc_read();  // Leitura descartada para estabilização
        uint16_t raw_x = adc_read();
        int filtered_x = applyMovingAverage(raw_x, buffer_x, &index_x, &sum_x);
        int scaled_x = scaleValue(filtered_x);

        // Leitura do eixo Y
        adc_select_input(ADC_CHANNEL_Y);
        (void)adc_read();  // Leitura descartada para estabilização
        uint16_t raw_y = adc_read();
        int filtered_y = applyMovingAverage(raw_y, buffer_y, &index_y, &sum_y);
        int scaled_y = scaleValue(filtered_y);

        // Preenche a mensagem para o joystick
        msg.type = TYPE_JOYSTICK;
        msg.data.joystick.x = scaled_x;
        msg.data.joystick.y = scaled_y;
        xQueueSend(xQueueMessage, &msg, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/*
 * Callback para os botões.
 * Pressionado = nível 0 (assumindo pull-up ativo).
 */
void button_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    message_t msg;
    
    // Verifica se o botão foi pressionado
    if (!gpio_get(gpio)) {
        msg.type = TYPE_BUTTON;
        if (gpio == BUTTON_A_PIN) {
            msg.data.button.key = 'A';
        } else if (gpio == BUTTON_D_PIN) {
            msg.data.button.key = 'D';
        }
        xQueueSendFromISR(xQueueMessage, &msg, &xHigherPriorityTaskWoken);
    }
    gpio_acknowledge_irq(gpio, events);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * Tarefa de transmissão via UART:
 * - Para mensagens JOYSTICK: envia um pacote de 6 bytes
 *   [0x00, msb(x), lsb(x), msb(y), lsb(y), EOP]
 * - Para mensagens BUTTON: envia um pacote de 3 bytes
 *   [0x01, key, EOP]
 */
void vTaskUART(void *p) {
    message_t msg;
    while (1) {
        if (xQueueReceive(xQueueMessage, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.type == TYPE_JOYSTICK) {
                uint8_t packet[6];
                packet[0] = 0x00; // Header de joystick
                packet[1] = (uint8_t)((msg.data.joystick.x >> 8) & 0xFF);
                packet[2] = (uint8_t)(msg.data.joystick.x & 0xFF);
                packet[3] = (uint8_t)((msg.data.joystick.y >> 8) & 0xFF);
                packet[4] = (uint8_t)(msg.data.joystick.y & 0xFF);
                packet[5] = EOP;
                uart_write_blocking(UART_ID, packet, 6);
            } else if (msg.type == TYPE_BUTTON) {
                uint8_t packet[3];
                packet[0] = 0x01; // Header de botão
                packet[1] = (uint8_t)msg.data.button.key;
                packet[2] = EOP;
                uart_write_blocking(UART_ID, packet, 3);
            }
        }
    }
}

int main() {
    stdio_init_all();
    adc_init();
    // Inicializa os canais do ADC (ex: GPIO26 e GPIO27)
    adc_gpio_init(26);
    adc_gpio_init(27);

    // Inicializa os pinos dos botões
    gpio_init(BUTTON_A_PIN);
    gpio_set_dir(BUTTON_A_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_A_PIN);
    
    gpio_init(BUTTON_D_PIN);
    gpio_set_dir(BUTTON_D_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_D_PIN);

    // Configura as interrupções dos botões
    gpio_set_irq_enabled_with_callback(BUTTON_A_PIN, GPIO_IRQ_EDGE_FALL, true, &button_callback);
    gpio_set_irq_enabled(BUTTON_D_PIN, GPIO_IRQ_EDGE_FALL, true); // Usa o mesmo callback

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    xQueueMessage = xQueueCreate(32, sizeof(message_t));
    if (xQueueMessage == NULL) {
        while(1);
    }
    
    xTaskCreate(vTaskADC, "Tarefa_ADC", 256, NULL, 1, NULL);
    xTaskCreate(vTaskUART, "Tarefa_UART", 256, NULL, 1, NULL);
    
    vTaskStartScheduler();
    
    while(1);
    return 0;
}