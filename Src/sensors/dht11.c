/*
 * dht11.c
 *
 *  Created on: 10 cze 2017
 *      Author: Spaqin
 */

#include "sensors/dht11.h"


const char humidity_title[] = "HUMDTY";
const char temperature_title[] = "TEMP";
const char temperature_pattern[] = "%d.%d°";
const char humidity_pattern[] = "%d.%d%%";

uint32_t _dht11_timing_data[41];
void dht11_init()
{
	_dht11_data.humidity_text[1] = '0';
	_dht11_data.humidity_text[2] = '.';
	_dht11_data.humidity_text[3] = '0';
	_dht11_data.humidity_text[4] = '%';
	_dht11_data.humidity_text[5] = ' ';
	_dht11_data.humidity_text[6] = '\0';
	_dht11_data.humidity_text[0] = ' ';
	Data_Display dht11_humidity_disp;
	dht11_humidity_disp.callback_after_title = _dht11_hum_title_callback;
	dht11_humidity_disp.title.no_dot = humidity_title;
	dht11_humidity_disp.data.dot = _dht11_data.humidity_text;
	dht11_humidity_disp.dot_info = DATA_DOT;
	dht11_humidity_disp.data_delay = 1024; //= 4s on a 32khz clock with 128 div prescaler
	dht11_humidity_disp.title_delay = 512; // 2s
	dht11_humidity_disp.level = 0;
	disp_mgr_register(dht11_humidity_disp);
	Data_Display dht11_temp_disp;
	dht11_temp_disp.callback_after_title = _dht11_temp_title_callback;
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
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //ok, it's not the perfect way. it blocks interrupts for up to ~4ms,
    //and it's hard-coded for the CPU speed
    //i *really* wanted to make it with a timer, input capture DMA
    //but i couldn't get it to work ;_;
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
    int i=400000;
    while(--i);
    __disable_irq();
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
    for(int j = 0; j < 41; ++j)
    {
    	int cnt = 0;
    	while(!(DHT11_GPIO_Port->IDR & DHT11_Pin))
    	{
    		cnt++;
    		if(cnt > 30000)
    		{
    			__enable_irq();
    			return;
    		}
    	}
    	cnt = 0;
    	while((DHT11_GPIO_Port->IDR & DHT11_Pin))
    	{
    		cnt++;
    		if(cnt > 30000)
    		{
    			__enable_irq();
    			return;
    		}
    	}
    	_dht11_timing_data[j] = cnt;
    }
    __enable_irq();
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
}

int _dht11_parse_results()
{
	uint32_t* bits = _dht11_timing_data + 1;
	uint8_t scratchpad[5] = { 0 };
	uint8_t sum = 0;
	for(int i = 0; i < 5; ++i)
	{
		for(int j = 0; j < 8; ++j)
		{
			scratchpad[i] |= bits[j] & 256 ? 1 : 0;
			scratchpad[i] <<= 1;
		}
		sum += scratchpad[i];
		bits += 8;
	}
	sum -= scratchpad[4];
	if(sum != scratchpad[4])
		return 1;
	_dht11_data.humidity_int = scratchpad[0];
	_dht11_data.humidity_dec = scratchpad[1];
	_dht11_data.temperature_int = scratchpad[2];
	_dht11_data.temperature_dec = scratchpad[3];
	return 0;
}

void _dht11_hum_title_callback()
{
	_dht11_init_measurement();
	if(_dht11_parse_results())
		return;
	char buffer[7];
	char* buf_it = buffer;
	int i;
	sprintf(buffer, humidity_pattern, _dht11_data.humidity_int, _dht11_data.humidity_dec);
	_dht11_data.humidity_text[0] = ' ';
	for(i = 0; *buf_it; buf_it++)
	{
		if(*buf_it != '.')
		{
			++i;
			_dht11_data.humidity_text[i] = *buf_it;

		}
		else
		{
			_dht11_data.humidity_text[i] |= DOT;
		}
	}
	_dht11_data.humidity_text[i+1] = 0;

}

void _dht11_temp_title_callback()
{
	_dht11_init_measurement();
	if(_dht11_parse_results())
		return;
	char buffer[7];
	char* buf_it = buffer;
	int i;
	sprintf(buffer, temperature_pattern, _dht11_data.temperature_int, _dht11_data.temperature_dec);
	_dht11_data.temperature_text[0] = ' ';
	for(i = 0; *buf_it; buf_it++)
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
	return _dht11_data.temperature_int;
}

uint16_t _dht11_get_humidity()
{
	return _dht11_data.humidity_int;
}

