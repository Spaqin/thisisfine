/* PM sensor library.
 * Holds the values, overrides UART interrupt handler.
 * communication protocol:
 * 	bit rate: 9600
 * 	data bits: 8
 * 	parity: nope
 * 	stop bits: 1
 *
 * */
#ifndef pmsensor
#define pmsensor
#include "stm32l4xx_hal.h"
#include "display_manager.h"

/* Struct describing values from the sensor for PM2.5 or PM10, averaged for smoothing
 * values are held in a cyclic buffer, overwritten when
 * */
typedef struct {
	uint16_t values[8];
	uint16_t average;
	Data_Display* disp_ptr;
} pm_data_t;

typedef struct {
	pm_data_t pm25_data;
	pm_data_t pm10_data;
	uint8_t current_ptr;  //ptr for both datas
	// bytes, in order:
	// 0xAA, 0xC0, PM2.5 low, PM2.5 high, PM10 low, PM10 high, serial hi, serial lo, checksum, 0xAB
	// buffer for the data, with an iterator
	uint8_t usart_data[10];
	uint8_t usart_data_ptr;
	uint32_t total_cycles; //debug info
	uint8_t* data_pattern;
	uint16_t* pm25_title;
	uint16_t* pm10_title;
	uint16_t pm25_data_string[7];
	uint16_t pm10_data_string[7];
	//remember about decimal point (div by 10 for value)
	uint16_t* pm25_thresholds;
	uint16_t* pm10_thresholds;
} all_pm_data_t;

all_pm_data_t all_pm_data;


uint16_t get_sum(pm_data_t*);
uint16_t get_avg(pm_data_t*);
uint16_t get_fast_avg(pm_data_t*);
float get_avgf(pm_data_t*);
void init_pm_data(UART_HandleTypeDef*);
void calculate_avg(pm_data_t*);
void pm_register_data_to_disp(void);
void pm25_display_callback(void);
void pm10_display_callback(void);
uint8_t parse_buffer_data(void);
#endif
