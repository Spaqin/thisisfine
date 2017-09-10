#include "comms/display_manager.h"

uint8_t _no_data[] = "NODATA";

/**
 * Initialization function.
 * Does LCD init and starts the timer.
 * @timer_handle - self explanatory;
 * @delay - it's in ticks, not ms, so you need to make your own calculations. it's the base for all the other delays too
 */
void disp_mgr_init(LPTIM_HandleTypeDef* timer_handle)
{
	_disp_mgr.disp_mgr_data_ptr = 0;
	_disp_mgr.title_or_data = TITLE;
	_disp_mgr.timer_handle = timer_handle;
	BSP_LCD_GLASS_Init();
	HAL_LPTIM_Counter_Start_IT(timer_handle, 128);
}

void disp_mgr_timer_handler(void)
{
	if(_disp_mgr.size == 0)
	{
		BSP_LCD_GLASS_DisplayString(_no_data);
		BSP_LCD_GLASS_BarLevelConfig(0);
		return;
	}
	int ptr = _disp_mgr.disp_mgr_data_ptr;
	Data_Display disp_data = _disp_mgr.disp_mgr_data[ptr];
	if(_disp_mgr.title_or_data == TITLE)
	{
		_disp_mgr.title_or_data = DATA;
		BSP_LCD_GLASS_Clear();
		if(disp_data.dot_info & TITLE_DOT)
			BSP_LCD_GLASS_DisplayStrDeci(disp_data.title.dot);
		else
			BSP_LCD_GLASS_DisplayString(disp_data.title.no_dot);
		BSP_LCD_GLASS_BarLevelConfig(0);
		//set new autoreload value
		_disp_mgr.timer_handle->State= HAL_LPTIM_STATE_BUSY;
		__HAL_LPTIM_AUTORELOAD_SET(_disp_mgr.timer_handle,disp_data.title_delay);
		_disp_mgr.timer_handle->State= HAL_LPTIM_STATE_READY;
		disp_data.callback_after_title();
	}
	else
	{
		_disp_mgr.title_or_data = TITLE;
		_disp_mgr.disp_mgr_data_ptr = ptr == _disp_mgr.size - 1 ? 0 : ptr + 1;
		BSP_LCD_GLASS_Clear();
		if(disp_data.dot_info & DATA_DOT)
			BSP_LCD_GLASS_DisplayStrDeci(disp_data.data.dot);
		else
			BSP_LCD_GLASS_DisplayString(disp_data.data.no_dot);
		BSP_LCD_GLASS_BarLevelConfig(disp_data.level);

		_disp_mgr.timer_handle->State= HAL_LPTIM_STATE_BUSY;
		__HAL_LPTIM_AUTORELOAD_SET(_disp_mgr.timer_handle,disp_data.data_delay);
		_disp_mgr.timer_handle->State= HAL_LPTIM_STATE_READY;

	}

}

Data_Display* disp_mgr_register(Data_Display to_insert)
{
	if(_disp_mgr.size == MAX_DISP_DATA)
		return NULL;
	_disp_mgr.disp_mgr_data[_disp_mgr.size] = to_insert;
	Data_Display* to_ret = &_disp_mgr.disp_mgr_data[_disp_mgr.size];
	_disp_mgr.size++;
	return to_ret;
}

