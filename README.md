# Controle de Jogo – Firmware

Este repositório contém o firmware para o controle de jogo desenvolvido para a plataforma Raspberry Pi Pico, utilizando FreeRTOS. O firmware gerencia a leitura de sensores, tratamento de entradas e a atualização de saídas visuais, bem como a comunicação serial via UART e HC-06 (Bluetooth).

## Jogo
O firmware foi desenvolvido para um controle que simula ações de "mirar" (botão esquerdo) e "atirar" (botão direito) em um ambiente de jogo. A ideia é utilizar entradas de sensores inerciais e outros dispositivos para interagir com o jogo.

## Ideia do Controle
- **Objetivo:** Oferecer uma interface para simular ações (como mirar e disparar) utilizando sensores e entradas físicas.
- **Funcionalidades:**  
  - Leitura de movimentos e orientação com o sensor MPU6050.
  - Controle de disparo e mira com botões físicos.
  - Feedback visual com LEDs RGB.
  - Comunicação sem fio com módulo HC-06 via UART para enviar dados do controle ao jogo.

## Inputs e Outputs
### Inputs (Sensores e Entradas)
- **Sensor Inercial (MPU6050):**  
  - Comunicação via I2C (pins SDA e SCL – pinos 4 e 5)
  - Captura de aceleração e giroscópio para determinar a orientação.
- **Encoder:**  
  - Entradas digitais para capturar a rotação (pinos 2 e 3) utilizando IRQ para detecção de mudanças.
- **Botões:**  
  - Botão A (boost): pino 17, acionado para "dar o boost".
  - Botão B (fire): pino 18, acionado para "atirar".

### Outputs (Atuadores)
- **LEDs RGB:**  
  - Dois conjuntos de LEDs controlados via PWM.  
  - LEDs 1 (vermelho, verde e azul) nos pinos 11, 12 e 13.
  - LEDs 2 (vermelho, verde e azul) nos pinos 14, 15 e 16.
- **Comunicação Serial:**  
  - UART para debug e transmissão dos dados de controle.
  - Módulo HC-06 para comunicação sem fio via Bluetooth (configurado nas mesmas UARTs).

## Protocolo Utilizado
- **I2C:**  
  - Comunicação entre a Pico e o sensor MPU6050 para leitura dos dados inerciais.
- **UART:**  
  - Duas interfaces UART:  
    - Uma para debug/comunicação com PC.
    - Outra para o módulo HC-06, possibilitando a conexão sem fio com o jogo.
- **PWM:**  
  - Controle dos LEDs RGB para feedback visual.

## Diagrama de Blocos do Firmware

```mermaid
flowchart TD
    %% Setup
    A[Setup & Inicialização] --> B[Configuração de IO]
    B --> C[Criação de Tasks]

    %% Definição do subgráfico
    subgraph "Tarefas FreeRTOS"
        C1[mpu_task]
        C2[uart_task]
        C3[led_task]
    END

    %% Conexão do grafo principal
    C --> C1
    C --> C2
    C --> C3

    %% Entradas (cada linha uma ligação)
    A1[MPU6050 - I2C] --> C1
    A2[Encoder - IRQ via GPIO] --> C1
    A3[Botões Aim e Fire - GPIO] --> C1

    %% Saídas
    C1 --> D[Filas]
    D --> C2
    C3 --> E[LEDs RGB - PWM]
    C2 --> F[UART/HC06]

    %% IRQs com estilo diferente
    G((Encoder IRQ Callback)) -.-> C1

    %% Links clicáveis (GitHub suporta click)
    click A1 "hardware/i2c.h" "Sensor MPU6050 via I2C"
    click E "hardware/pwm.h" "Controle dos LEDs RGB"
    click F "hardware/uart.h" "Comunicação via UART / HC06"
```