#ifndef display_manager
#define display_manager
#include "stm32l4xx_hal.h"
#include "stm32l476g_discovery_glass_lcd.h"
#define MAX_DISP_DATA 16
#define TITLE 0
#define DATA 1
#define TITLE_DOT 0x1
#define DATA_DOT 0x2

typedef union {
	uint8_t* no_dot;
	uint16_t* dot;
} Char_Union_Typedef;

/*
 * This struct defines what will be displayed on the screen.
 * The principle of Display Manager is that it goes, in a loop, over every registered data display,
 * the title first, then callback which has to prepare the data (and level if applicable) in the delay time,
 * then finally shows the prepared data.
 * */
typedef struct{
	Char_Union_Typedef title;
	Char_Union_Typedef data;
	void (*callback_after_title)(void);
	BatteryLevel_Typedef level;
	uint8_t dot_info;
	uint16_t title_delay;
	uint16_t data_delay;
}Data_Display;

typedef struct{
	Data_Display disp_mgr_data[MAX_DISP_DATA];
	uint16_t disp_mgr_data_ptr;
	uint16_t size;
	uint8_t title_or_data;
	LPTIM_HandleTypeDef* timer_handle;
}_Display_Manager_Typedef;

_Display_Manager_Typedef _disp_mgr;


void disp_mgr_init(LPTIM_HandleTypeDef* timer_handle);
void disp_mgr_timer_handler(void);
Data_Display* disp_mgr_register(Data_Display);

#endif
