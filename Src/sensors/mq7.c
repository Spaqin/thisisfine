/*
 * mq7.c
 *
 *  Created on: 15 lip 2017
 *      Author: Spaqin
 */

#include "sensors/mq7.h"

const char mq7_title[] = "CO";
const char mq7_pattern[] = "%dppm";
ADC_HandleTypeDef* mq7_adch;

void mq7_init(ADC_HandleTypeDef* adch)
{
	Data_Display mq7_disp;
	mq7_disp.callback_after_title = _mq7_callback;
	mq7_disp.title.no_dot = mq7_title;
	mq7_disp.data.no_dot = _mq7_data.co_text;
	mq7_disp.dot_info = NO_DOTS;
	mq7_disp.data_delay = 1024; //= 4s on a 32khz clock with 128 div prescaler
	mq7_disp.title_delay = 512; // 2s
	mq7_disp.level = 0;
	disp_mgr_register(mq7_disp);
	fineproto_add_sensor(_mq7_get_value, CO);
	HAL_ADC_Start_DMA(adch, (uint8_t*) _mq7_data.raw_adc, 8);
	mq7_adch = adch;
}


void _mq7_callback()
{
	uint16_t avg = 0;
	for(int i = 0; i < 8; ++i)
	{
		avg += _mq7_data.raw_adc[i];
	}
	avg >>= 3;
	uint16_t  voltage = 3300 * avg/4095;
	_mq7_data.co_voltage = voltage;
	// OK. This is not PPM. The problem is that the only thing with this sensor's datasheet you get,
	// is ratio of resistance at current level and at 100ppm. I don't know either, so this will require further investigation.
	// Known data, to be exact. This is also why I don't include a level indicator.
	sprintf(_mq7_data.co_text, mq7_pattern, voltage);
}


uint16_t _mq7_get_value()
{
	return _mq7_data.co_voltage;
}
