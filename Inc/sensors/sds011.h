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
#include "comms/display_manager.h"

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

	uint32_t total_cycles; //debug info
	const uint8_t* data_pattern;
	const uint16_t* pm25_title;
	const uint16_t* pm10_title;
	uint16_t pm25_data_string[7];
	uint16_t pm10_data_string[7];
	//remember about decimal point (div by 10 for value)
	const uint16_t* pm25_thresholds;
	const uint16_t* pm10_thresholds;
} sds011_data_t;

sds011_data_t all_pm_data;

uint8_t sds_last_usart_byte;
uint8_t sds_usart_data[4];
uint8_t sds_buffered_data[4];

uint16_t get_sum(pm_data_t*);
uint16_t get_avg(pm_data_t*);
uint16_t get_fast_avg(pm_data_t*);
float get_avgf(pm_data_t*);
void init_pm_data(UART_HandleTypeDef*);
void calculate_avg(pm_data_t*);
void pm_register_data_to_disp(void);
void pm25_display_callback(void);
void pm10_display_callback(void);
uint8_t sds011_parse_buffer_data(void);
uint16_t _sds011_get_pm25(void);
uint16_t _sds011_get_pm10(void);
#endif
