#include "stm32l4xx_hal.h"
#include <string.h>
#define HM10_UART huart2

extern UART_HandleTypeDef HM10_UART;

void hm10_init();  //start connection, set name, start working
void hm10_send_AT_message(char*);  // blocking
void hm10_send_message(uint8_t*, uint32_t);  // non-blocking, using DMA
