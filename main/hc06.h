#ifndef HC06_H_
#define HC06_H_

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <string.h>

#include "pico/stdlib.h"
#include <stdio.h>

#define HC06_UART_ID uart1
#define HC06_BAUD_RATE 115200
#define HC06_STATE_PIN 10
#define HC06_TX_PIN 8
#define HC06_RX_PIN 9
#define HC06_ENABLE_PIN 6

bool hc06_check_connection();
bool hc06_set_name(char name[]);
bool hc06_set_pin(char pin[]);
bool hc06_set_at_mode(int on);
bool hc06_init(char name[], char pin[]);


#endif // HC06_H_