#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <stdio.h>
#include <string.h>

// Configurações do UART (utilizando UART0 como exemplo)
#define UART_ID         uart0
#define BAUD_RATE       115200
#define UART_TX_PIN     0      // Por exemplo, GP0
#define UART_RX_PIN     1      // Por exemplo, GP1

// Configurações do HC-06
#define HC06_UART        uart1            // Usaremos a UART1
#define HC06_BAUD_RATE   9600             // Baud rate padrão do HC-06

// Pinos para UART1 na Pico (exemplo comum para a Pico W)
#define HC06_UART_TX_PIN 4                // Pico TX (UART1) → ligado ao HC-06 RX
#define HC06_UART_RX_PIN 5                // Pico RX (UART1) ← ligado ao HC-06 TX

// Pinos de controle do módulo HC-06
#define HC06_STATE_PIN   2                // Estado do bluetooth (opcional)
#define HC06_EN_PIN      3                // Pino Enable (ativo em nível baixo)

// Pinos para o ADC (conectados aos eixos do joystick)
#define ADC_CHANNEL_X   0      // GPIO26 (ADC0)
#define ADC_CHANNEL_Y   1      // GPIO27 (ADC1)

// Valor central do ADC para 12 bits: aproximadamente 2048
#define ADC_CENTRAL     2048
// Limiar para considerar movimento (zona morta)
#define DEADZONE        1250    // Ajuste conforme necessário

// Delay simples (você pode usar sleep_ms() diretamente)
static inline void delay_ms_custom(uint32_t ms) {
    sleep_ms(ms);
}

// Função para enviar um comando AT ao HC-06
void hc06_send_at_command(const char *cmd) {
    uart_puts(HC06_UART, cmd);
    uart_puts(HC06_UART, "\r\n");
    delay_ms_custom(200);  // Aguarda 200 ms pela resposta
}

// Função de inicialização do HC-06, configurando nome e PIN
void hc06_init(const char *name, const char *pin) {
    // Envia comando AT básico para testar a conexão
    uart_puts(HC06_UART, "AT\r\n");
    delay_ms_custom(500);
    
    // Configura o nome do módulo
    char at_cmd[64];
    sprintf(at_cmd, "AT+NAME=%s", name);
    hc06_send_at_command(at_cmd);
    
    // Configura o PIN do módulo
    sprintf(at_cmd, "AT+PIN=%s", pin);
    hc06_send_at_command(at_cmd);
}

void init_uart(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void init_adc(void) {
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
}

uint16_t ler_adc(uint8_t canal) {
    adc_select_input(canal);
    return adc_read();
}

int main() {
    // Inicializa o stdio (por exemplo, para debug via USB serial)
    stdio_init_all();
    
    // Inicializa a UART1 com a baud rate definida para o HC-06
    // uart_init(HC06_UART, HC06_BAUD_RATE);
    // gpio_set_function(HC06_UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(HC06_UART_RX_PIN, GPIO_FUNC_UART);
    
    // Configura os pinos de controle
    // gpio_init(HC06_STATE_PIN);
    // gpio_set_dir(HC06_STATE_PIN, GPIO_IN);
    
    // gpio_init(HC06_EN_PIN);
    // gpio_set_dir(HC06_EN_PIN, GPIO_OUT);
    // Conforme o exemplo, coloque 0 em EN para habilitar o módulo
    // gpio_put(HC06_EN_PIN, 0);

    init_uart();
    init_adc();
    
    // Inicializa o módulo HC-06 com o nome e PIN desejados
    // (Exemplo: nome "aps2_legal" e PIN "1234")
    // hc06_init("aps2_legal", "1234");
    
    char mensagem[64];
    while (true) {
        uint16_t adc_x = ler_adc(ADC_CHANNEL_X);
        uint16_t adc_y = ler_adc(ADC_CHANNEL_Y);

        int dx = (int)adc_x - ADC_CENTRAL;
        int dy = (int)adc_y - ADC_CENTRAL;

        if(abs(dx) < DEADZONE) dx = 0;
        if(abs(dy) < DEADZONE) dy = 0;

        // sprintf(mensagem, "abs dy: %d", abs(dy));

        int x = dx / 50;
        int y = dy / 50;

        // Monte a mensagem para envio. Formato: ANALOG,x,y\n
        sprintf(mensagem, "ANALOG,%d,%d\r\n", x, y);

        // Envia a mensagem via UART para o computador
        uart_puts(UART_ID, mensagem);

        sleep_ms(5);  // Envia a cada 50 ms (ajuste conforme necessário)

        // Envia a string de teste "OLAAA " via UART para o HC-06
        // uart_puts(HC06_UART, "OLAAA ");
        
        // Se desejar, você pode ler o pino STATE para verificar se está conectado
        // Exemplo:
        // bool connected = gpio_get(HC06_STATE_PIN);
        // printf("Estado Bluetooth: %s\n", connected ? "Conectado" : "Desconectado");
        
        // sleep_ms(100);  // Aguarda 100 ms entre os envios
    }
    
    return 0;
}