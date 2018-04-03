/*
 * dht11.h
 *
 *  Created on: 10 cze 2017
 *      Author: Spaqin
 */

#ifndef DHT11_H_
#define DHT11_H_

#include "stm32l4xx_hal.h"
#include "comms/display_manager.h"
#include "comms/fineproto.h"

#define DHT_DOWN_TIME 20000

extern TIM_HandleTypeDef htim4;

typedef struct {
	uint8_t humidity_int;
	uint8_t humidity_dec;
	uint8_t temperature_int;
	uint8_t temperature_dec;
	uint16_t humidity_text[7];
	uint16_t temperature_text[7];
} DHT11_t;

DHT11_t _dht11_data;
uint32_t _dht11_timing_data[43];
uint8_t dht_i;

int dht11_get_humidity();
int dht11_get_temperature();
void dht11_init();
void _dht11_init_measurement();
int _dht11_parse_results();
void _dht11_hum_title_callback();
void _dht11_temp_title_callback();
void _dht11_dma_callback();
uint16_t _dht11_get_temp();
uint16_t _dht11_get_humidity();


#endif /* DHT11_H_ */
