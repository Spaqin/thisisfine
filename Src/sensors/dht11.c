/*
 * dht11.c
 *
 *  Created on: 10 cze 2017
 *      Author: Spaqin
 */

#include "sensors/dht11.h"


const char humidity_title[] = "HUMDTY";
const char temperature_title[] = "TEMP";
const char temperature_pattern[] = "%d.%d°C";
const char humidity_pattern[] = "%d.%d%%";


void dht11_init()
{
//	GPIO_InitTypeDef GPIO_InitStruct;
//    GPIO_InitStruct.Pin = DHT_IC_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    HAL_GPIO_Init(DHT_IC_GPIO_Port, &GPIO_InitStruct);
////    GPIO_InitStruct.Pin = DHT_IC_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//
//    HAL_GPIO_WritePin(DHT_IC_GPIO_Port, DHT_IC_Pin, GPIO_PIN_SET);
	_dht11_data.humidity_text[1] = '-' | DOT;
	_dht11_data.humidity_text[2] = '-';
	_dht11_data.humidity_text[3] = '%';
	_dht11_data.humidity_text[4] = ' ';
	_dht11_data.humidity_text[5] = '\0';
	_dht11_data.humidity_text[0] = ' ';
	_dht11_data.temperature_text[1] = '-' | DOT;
	_dht11_data.temperature_text[2] = '-';
	_dht11_data.temperature_text[3] = '°';
	_dht11_data.temperature_text[4] = 'C';
	_dht11_data.temperature_text[5] = '\0';
	_dht11_data.temperature_text[0] = ' ';
	Data_Display dht11_humidity_disp;
	dht11_humidity_disp.callback_after_title = _dht11_init_measurement;
	dht11_humidity_disp.title.no_dot = humidity_title;
	dht11_humidity_disp.data.dot = _dht11_data.humidity_text;
	dht11_humidity_disp.dot_info = DATA_DOT;
	dht11_humidity_disp.data_delay = 1024; //= 4s on a 32khz clock with 128 div prescaler
	dht11_humidity_disp.title_delay = 512; // 2s
	dht11_humidity_disp.level = 0;
	disp_mgr_register(dht11_humidity_disp);
	Data_Display dht11_temp_disp;
	dht11_temp_disp.callback_after_title = _dht11_init_measurement;
	dht11_temp_disp.title.no_dot = temperature_title;
	dht11_temp_disp.data.dot = _dht11_data.temperature_text;
	dht11_temp_disp.dot_info = DATA_DOT;
	dht11_temp_disp.data_delay = 1024; //= 4s on a 32khz clock with 128 div prescaler
	dht11_temp_disp.title_delay = 512; // 2s
	dht11_temp_disp.level = 0;
	disp_mgr_register(dht11_temp_disp);
	fineproto_add_sensor(_dht11_get_temp, Temperature);
	fineproto_add_sensor(_dht11_get_humidity, Humidity);

}

void _dht11_init_measurement()
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = DHT_IC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_IC_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(DHT_IC_GPIO_Port, DHT_IC_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) + DHT_DOWN_TIME));
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
}

int _dht11_parse_results()
{
	//find the offset
	uint8_t offset = 0;
	for(int i = 0; i < 40; ++i)
	{
		if(_dht11_timing_data[i] > 135 && _dht11_timing_data[i] < 200) //it's from 140 to 180
		{
			offset = i;
			break;
		}
	}
	uint32_t* bits = _dht11_timing_data + offset + 1;
	uint8_t scratchpad[5] = { 0 };
	uint8_t sum = 0;
	for(int i = 0; i < 5; ++i)
	{
		for(int j = 0; j < 8; ++j)
		{
			//timings are really well kept
			scratchpad[i] <<= 1;
			scratchpad[i] |= bits[j+i*8] > 100 ? 1 : 0;
		}
		sum += scratchpad[i];
	}
	sum -= scratchpad[4];
	if(sum != scratchpad[4])
		return 1;
	_dht11_data.humidity_int = scratchpad[0];
	_dht11_data.humidity_dec = scratchpad[1];
	_dht11_data.temperature_int = scratchpad[2];
	_dht11_data.temperature_dec = scratchpad[3];
	_dht11_hum_title_callback();
	_dht11_temp_title_callback();
	return 0;
}

void _dht11_hum_title_callback()
{
	char buffer[7];
	char* buf_it = buffer;
	int i;
	sprintf(buffer, humidity_pattern, _dht11_data.humidity_int, _dht11_data.humidity_dec);
	for(i = 0; *buf_it; buf_it++)
	{
		if(*buf_it != '.') {
			_dht11_data.humidity_text[i++] = *buf_it;
		}
		else {
			_dht11_data.humidity_text[i] |= DOT;
		}
	}
	_dht11_data.humidity_text[i+1] = 0;

}

void _dht11_temp_title_callback()
{
	char buffer[7];
	char* buf_it = buffer;
	int i;
	sprintf(buffer, temperature_pattern, _dht11_data.temperature_int, _dht11_data.temperature_dec);
	for(i = -1; *buf_it; buf_it++)
	{
		if(*buf_it != '.')
		{
			++i;
			_dht11_data.temperature_text[i] = *buf_it;
		}
		else
		{
			_dht11_data.temperature_text[i] |= DOT;
		}
	}
	_dht11_data.temperature_text[i+1] = 0;

}

uint16_t _dht11_get_temp()
{
	return _dht11_data.temperature_int | _dht11_data.temperature_dec << 8;
}

uint16_t _dht11_get_humidity()
{
	return _dht11_data.humidity_int | _dht11_data.humidity_dec << 8;
}

