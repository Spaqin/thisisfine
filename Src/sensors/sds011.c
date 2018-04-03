/*
 * pmsensor.c
 *
 *  Created on: 8 maj 2017
 *      Author: Spaqin
 */
#include "sensors/sds011.h"
#include "comms/fineproto.h"

const uint8_t _data_pattern[] = "%d";
const uint16_t _pm25_title[] = {'P', 'M', '2' | DOT, '5' | DOUBLE_DOT, ' ', ' '};
const uint16_t _pm10_title[] = {'P', 'M', '1', '0' | DOUBLE_DOT, ' ', ' '};
// danger levels in ug/m3
// pm2.5
const uint16_t _pm25_thresholds[] = {100, 200, 500, 1000};
const uint16_t _pm10_thresholds[] = {300, 500, 2000, 3000};
uint16_t get_sum(pm_data_t* pm_data){
	uint16_t sum = 0;
	int i;
	for(i = 0; i < all_pm_data.total_cycles && i < 8; ++i)
	{
		sum += pm_data->values[i];
	}
	return sum;
}


uint16_t get_avg(pm_data_t* pm_data) {
	uint16_t sum = get_sum(pm_data);
	if (all_pm_data.total_cycles >= 8)
		return sum>>3;
	return sum/all_pm_data.total_cycles;
}

float get_avgf(pm_data_t* pm_data) {
	float sum = get_sum(pm_data);
	if (all_pm_data.total_cycles >= 8)
		return sum/80.0f;
	return sum/(all_pm_data.total_cycles*10.0f);
}

void init_pm_data(UART_HandleTypeDef* huart) {
	all_pm_data.current_ptr = 0;
	all_pm_data.total_cycles = 0;
	//all_pm_data.usart_data_ptr = 0;
	all_pm_data.data_pattern = _data_pattern;
	all_pm_data.pm25_title = _pm25_title;
	all_pm_data.pm10_title = _pm10_title;
	all_pm_data.pm25_thresholds = _pm25_thresholds;
	all_pm_data.pm10_thresholds = _pm10_thresholds;
	pm_register_data_to_disp();
	fineproto_add_sensor(_sds011_get_pm25, PM25);
	fineproto_add_sensor(_sds011_get_pm10, PM10);
	HAL_UART_Receive_IT(huart, &sds_last_usart_byte, 1);
}

void calculate_avg(pm_data_t* pm_data){
	pm_data->average = get_avg(pm_data);
}

// 0xAA, 0xC0, PM2.5 low, PM2.5 high, PM10 low, PM10 high, serial hi, serial lo, checksum, 0xAB
// checksum = sum(pm2.5 low ~ serial lo)
// maybe restart DMA transmission on error?
uint8_t sds011_parse_buffer_data(void){
//	if(all_pm_data.usart_data[0] != 0xAA || all_pm_data.usart_data[1] != 0xC0 || all_pm_data.usart_data[9] != 0xAB)
//		return 1;
//	uint8_t checksum = 0;
//	checksum += all_pm_data.usart_data[2]; //pm2.5 lo
//	checksum += all_pm_data.usart_data[3]; //pm2.5 hi
//	checksum += all_pm_data.usart_data[4]; //pm10 lo
//	checksum += all_pm_data.usart_data[5]; //pm10 hi
//	checksum += all_pm_data.usart_data[6]; //serial hi
//	checksum += all_pm_data.usart_data[7]; //serial lo
//	if(checksum != all_pm_data.usart_data[8])
//		return 1;
//	uint8_t pm25_lo = all_pm_data.usart_data[2];
//	uint8_t pm25_hi = all_pm_data.usart_data[3];
//	uint8_t pm10_lo = all_pm_data.usart_data[4];
////	uint8_t pm10_hi = all_pm_data.usart_data[5];
	uint16_t pm25_value = (sds_usart_data[1]<<8) | sds_usart_data[0];
	uint16_t pm10_value = (sds_usart_data[3]<<8) | sds_usart_data[2];


	all_pm_data.pm25_data.values[all_pm_data.current_ptr] = pm25_value;
	all_pm_data.pm10_data.values[all_pm_data.current_ptr] = pm10_value;
	calculate_avg(&(all_pm_data.pm25_data));
	calculate_avg(&(all_pm_data.pm10_data));
	all_pm_data.current_ptr++;
	all_pm_data.current_ptr &= 0x7;
	all_pm_data.total_cycles++;
	return 0;
}

void pm_register_data_to_disp(void)
{
	Data_Display pm25_disp;
	Data_Display pm10_disp;
	pm25_disp.callback_after_title = pm25_display_callback;
	pm25_disp.title.dot = all_pm_data.pm25_title;
	pm25_disp.data.dot = all_pm_data.pm25_data_string;
	pm25_disp.dot_info = TITLE_DOT | DATA_DOT;
	pm25_disp.data_delay = 1024; //= 4s on a 32khz clock with 128 div prescaler
	pm25_disp.title_delay = 512; // 2s
	if((all_pm_data.pm25_data.disp_ptr = disp_mgr_register(pm25_disp)))
	{
		pm10_disp.callback_after_title = pm10_display_callback;
		pm10_disp.title.dot = all_pm_data.pm10_title;
		pm10_disp.data.dot = all_pm_data.pm10_data_string;
		pm10_disp.dot_info = TITLE_DOT | DATA_DOT;
		pm10_disp.data_delay = 1024; //= 2s on a 32khz clock with 128 div prescaler
		pm10_disp.title_delay = 512; // 1s
		all_pm_data.pm10_data.disp_ptr = disp_mgr_register(pm10_disp);
	}
}

void pm25_display_callback(void)
{
	char temp_buf[7];
	uint8_t buf_ptr = 6;
	uint8_t buf_size = 0;
	calculate_avg(&all_pm_data.pm25_data);

	sprintf(temp_buf, "%d", all_pm_data.pm25_data.average);
	//clean buffer
	while(buf_ptr--)
		all_pm_data.pm25_data_string[buf_ptr] = ' ';
	//determine temp buffer size
	while(temp_buf[++buf_size]);
	//write data string backwards
	buf_ptr = 5;
	++buf_size;
	while(buf_size)
	{
		all_pm_data.pm25_data_string[buf_ptr] = temp_buf[buf_size-1];
		buf_ptr--; buf_size--;
	}
	//thanks to left align, this is constant
	//[6] = \0, [5] is space, [4] is after dot, [3] is before dot
	all_pm_data.pm25_data_string[3] |= DOT;
	//and now set the "battery" level for the threshold
	Data_Display* dsp = all_pm_data.pm25_data.disp_ptr;
	for(dsp->level = 4; dsp->level; --dsp->level)
	{
		if(all_pm_data.pm25_data.average > all_pm_data.pm25_thresholds[dsp->level - 1])
			break;
	}
}

void pm10_display_callback(void)
{
	char temp_buf[7];
	uint8_t buf_ptr = 6;
	uint8_t buf_size = 0;
	calculate_avg(&all_pm_data.pm10_data);

	sprintf(temp_buf, "%d", all_pm_data.pm10_data.average);
	while(buf_ptr--)
		all_pm_data.pm10_data_string[buf_ptr] = ' ';
	while(temp_buf[++buf_size]);
	buf_ptr = 5;
	buf_size++;
	while(buf_size)
	{
		all_pm_data.pm10_data_string[buf_ptr] = temp_buf[buf_size-1];
		buf_ptr--; buf_size--;
	}
	all_pm_data.pm10_data_string[3] |= DOT;
	Data_Display* dsp = all_pm_data.pm10_data.disp_ptr;
	for(dsp->level = 4; dsp->level; --dsp->level)
	{
		if(all_pm_data.pm10_data.average > all_pm_data.pm10_thresholds[dsp->level - 1])
			break;
	}
}


uint16_t _sds011_get_pm25()
{
	return all_pm_data.pm25_data.average;
}


uint16_t _sds011_get_pm10()
{
	return all_pm_data.pm10_data.average;
}


