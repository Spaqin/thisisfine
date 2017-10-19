/*
 * hm10.c
 *
 *  Created on: 7 wrz 2017
 *      Author: Spaqin
 */

#include "comms/hm10.h"

void hm10_init()
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = BT_ENABLE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BT_ENABLE_GPIO_Port, &GPIO_InitStruct);
	if(HAL_GPIO_ReadPin(BT_ENABLE_GPIO_Port, BT_ENABLE_Pin))
		return;  //if state is high -> there's something connected already and init isn't needed
	// set BT_ENABLE GPIO state low and then high to restart the module
	// start UART

	hm10_send_AT_message("NAMEThisIsFine");
	hm10_send_AT_message("START");
	//start connection, set name, start working
}

void hm10_send_AT_message(char* msg)
{
	char buffer[32];
	char pattern[] = "AT+%s\r\n";
	sprintf(buffer, pattern, msg); //could be done manually to be faster
	uint32_t len = strlen(msg) + 5;
	// actually send message
	HAL_UART_Transmit(&HM10_UART, (uint8_t*) buffer, len, 40000);
}

uint32_t hm10_send_message(uint8_t* msg, uint32_t size)
{
	if(!HAL_GPIO_ReadPin(BT_ENABLE_GPIO_Port, BT_ENABLE_Pin))
		return 1;
	return HAL_UART_Transmit_DMA(&HM10_UART, msg, size);
	// non-blocking, using DMA
}
