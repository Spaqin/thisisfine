/*
 * dht11.h
 *
 *  Created on: 10 cze 2017
 *      Author: Spaqin
 */

#ifndef DHT11_H_
#define DHT11_H_

#include "stm32l4xx_hal.h"
#include "display_manager.h"

typedef struct {
	uint8_t humidity_int;
	uint8_t humidity_dec;
	uint8_t temperature_int;
	uint8_t temperature_dec;
	uint16_t humidity_text[7];
	uint16_t temperature_text[7];
} DHT11_t;

DHT11_t _dht11_data;

int dht11_get_humidity();
int dht11_get_temperature();
void dht11_init();
void _dht11_init_measurement();
int _dht11_parse_results();
void _dht11_hum_title_callback();
void _dht11_temp_title_callback();
void _dht11_dma_callback();


#endif /* DHT11_H_ */
