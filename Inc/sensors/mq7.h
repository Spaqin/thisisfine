/*
 * mq7.h
 *
 *  Created on: 15 lip 2017
 *      Author: Spaqin
 */

#ifndef MQ7_H_
#define MQ7_H_

#include "dht11.h"
#include "comms/display_manager.h"

typedef struct {
	uint16_t raw_adc[8];
	uint16_t co_voltage;
	int8_t co_text[7];
} mq7_t;

mq7_t _mq7_data;

void mq7_init(ADC_HandleTypeDef*);
void _mq7_callback(void);
uint16_t _mq7_get_value(void);

#endif /* MQ7_H_ */
