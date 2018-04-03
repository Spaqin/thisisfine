/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"

/* USER CODE BEGIN 0 */
#include "comms/display_manager.h"
#include "comms/fineproto.h"
#include "sensors/sds011.h"
#include "sensors/dht11.h"

#define UART_TIMEOUT 5

typedef enum {
	SDS_START, SDS_HEADER, SDS_CMD,
	SDS_PM25DATALOW, SDS_PM25DATAHI, SDS_PM10DATALOW, SDS_PM10DATAHI,
	SDS_SERIAL0, SDS_SERIAL1, SDS_CHKSUM, SDS_TAIL
}SDS_state;

SDS_state sds_state;

typedef enum {
	FP_START, FP_HEADER, FP_CMD, FP_D0, FP_D1, FP_CHKSUM
}FineProto_State;

FineProto_State fp_state;

extern uint8_t flags;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern LPTIM_HandleTypeDef hlptim1;
extern LPTIM_HandleTypeDef hlptim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel7 global interrupt.
*/
void DMA1_Channel7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */

  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */

  /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
* @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
*/
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_CC_IRQn 0 */

  /* USER CODE END TIM1_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_CC_IRQn 1 */
  HAL_PWR_EnableSleepOnExit();
  /* USER CODE END TIM1_CC_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles UART4 global interrupt.
*/
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
* @brief This function handles DMA2 channel3 global interrupt.
*/
void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */

  /* USER CODE END DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Channel3_IRQn 1 */

  /* USER CODE END DMA2_Channel3_IRQn 1 */
}

/**
* @brief This function handles DMA2 channel5 global interrupt.
*/
void DMA2_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel5_IRQn 0 */

  /* USER CODE END DMA2_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA2_Channel5_IRQn 1 */

  /* USER CODE END DMA2_Channel5_IRQn 1 */
}

/**
* @brief This function handles LPTIM1 global interrupt.
*/
void LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM1_IRQn 0 */

  /* USER CODE END LPTIM1_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim1);
  /* USER CODE BEGIN LPTIM1_IRQn 1 */

  /* USER CODE END LPTIM1_IRQn 1 */
}

/**
* @brief This function handles LPTIM2 global interrupt.
*/
void LPTIM2_IRQHandler(void)
{
  /* USER CODE BEGIN LPTIM2_IRQn 0 */

  /* USER CODE END LPTIM2_IRQn 0 */
  HAL_LPTIM_IRQHandler(&hlptim2);
  /* USER CODE BEGIN LPTIM2_IRQn 1 */

  /* USER CODE END LPTIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

uint8_t sds_tm = 0;
uint8_t fp_tm = 0;

inline void SDS_Timeout_Start() {
	sds_tm = 1;
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)(__HAL_TIM_GET_COUNTER(&htim1) + UART_TIMEOUT));
	//__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);

}

inline void SDS_Timeout_Stop() {
	if(sds_tm) {
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	}
	sds_tm = 0;
}



inline void FP_Timeout_Start() {
	fp_tm = 1;
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint16_t)(__HAL_TIM_GET_COUNTER(&htim1) + UART_TIMEOUT));
	//__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
}

inline void FP_Timeout_Stop() {
	if(fp_tm) {
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
		TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	}
	fp_tm = 0;
}

uint8_t usart2log[256];
uint8_t usart2i;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == UART4)
	{
		static uint8_t sds_checksum;
		SDS_Timeout_Stop();
		uint8_t sds_temp_b = sds_last_usart_byte;
		switch(sds_state)
		{
			case SDS_START:
				if(sds_temp_b == 0xAA)
				{
					sds_state = SDS_HEADER;
					SDS_Timeout_Start();
				}
				break;
			case SDS_HEADER:
				if(sds_temp_b == 0xC0)
				{
					sds_state = SDS_CMD;
				}
				else if(sds_temp_b != 0xAA)
				{
					sds_state = SDS_START;
					break;
				}
				SDS_Timeout_Start();
				break;
			case SDS_CMD:
				sds_usart_data[0] = sds_temp_b;
				sds_checksum = sds_temp_b;
				sds_state = SDS_PM25DATALOW;
				SDS_Timeout_Start();
				break;
			case SDS_PM25DATALOW:
				sds_state = SDS_PM25DATAHI;
				sds_usart_data[1] = sds_temp_b;
				sds_checksum += sds_temp_b;
				SDS_Timeout_Start();
				break;
			case SDS_PM25DATAHI:
				sds_state = SDS_PM10DATALOW;
				sds_usart_data[2] = sds_temp_b;
				sds_checksum += sds_temp_b;
				SDS_Timeout_Start();
				break;
			case SDS_PM10DATALOW:
				sds_state = SDS_PM10DATAHI;
				sds_usart_data[3] = sds_temp_b;
				sds_checksum += sds_temp_b;
				SDS_Timeout_Start();
				break;
			case SDS_PM10DATAHI:
				sds_state = SDS_SERIAL0;
				sds_checksum += sds_temp_b;
				SDS_Timeout_Start();
				break;
			case SDS_SERIAL0:
				sds_state = SDS_SERIAL1;
				sds_checksum += sds_temp_b;
				SDS_Timeout_Start();
				break;
			case SDS_SERIAL1:
				if(sds_temp_b != sds_checksum) {
					sds_state = SDS_START;
					break;
				}
				sds_state = SDS_CHKSUM;
				SDS_Timeout_Start();
				break;
			case SDS_CHKSUM:
				if(sds_temp_b == 0xAB)
				{
					memcpy(sds_buffered_data, sds_usart_data, 4);
					flags |= _FL_SDS011_MSG_RCV;
				}
				else if(sds_temp_b == 0xAA)
				{
					sds_state = SDS_HEADER;
					break;
				}
				/* no break */
			default:
				sds_state = SDS_START;
				//HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);

		}
		HAL_UART_Receive_IT(huart, &sds_last_usart_byte, 1);

//		if(sds011_parse_buffer_data()) //restart transmission if something went wrong
//		{
//			HAL_UART_DMAStop(huart);
//			HAL_UART_Receive_DMA(huart, all_pm_data.usart_data, 10);
//		}
	}
	else if(huart->Instance == USART2)
	{
		uint8_t fp_temp = fp_last_byte;
		usart2log[usart2i++] = fp_temp;
		FP_Timeout_Stop();
		switch(fp_state)
		{
			case FP_START:
				if(fp_temp == FINEPROTO_QUERY)
				{
					fp_state = FP_HEADER;
					fp_message_data_scratch[0] = fp_temp;
					fp_message_data_scratch[4] = fp_temp;
				}
				FP_Timeout_Start();
				break;
			// next 3 could be replaced with "one" tbh
			case FP_HEADER:
				fp_state = FP_CMD;
				fp_message_data_scratch[1] = fp_temp;
				fp_message_data_scratch[4] += fp_temp;
				FP_Timeout_Start();
				break;
			case FP_CMD:
				fp_state = FP_D0;
				fp_message_data_scratch[2] = fp_temp;
				fp_message_data_scratch[4] += fp_temp;
				FP_Timeout_Start();
				break;
			case FP_D0:
				fp_state = FP_D1;
				fp_message_data_scratch[3] = fp_temp;
				fp_message_data_scratch[4] += fp_temp;
				FP_Timeout_Start();
				break;
			case FP_D1:
				//D1 was received already, check the sum now
				if(fp_temp == fp_message_data_scratch[4])
				{
					memcpy(&(_fineproto.last_rcv), fp_message_data_scratch, 5);
					_fp_queue_message();
					flags |= _FL_BT_MSG_QUEUE;
				}
				/* no break */
			default:
				fp_state = FP_START;
		}
		HAL_UART_Receive_IT(huart, &fp_last_byte, 1);
//		if(_fp_queue_message())  <- life could be so much simpler
//		{
//			HAL_UART_DMAStop(huart);
//			HAL_UART_Receive_DMA(huart, (uint8_t*) &_fineproto.last_rcv, 5);
//		}
	}
}
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{

	/*
	 * Timeout support for UART FSMs.
	 * */
	if(htim->Instance == TIM1 &&
			htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		sds_state = SDS_START;
		SDS_Timeout_Stop();
		__set_PRIMASK(primask);
	}
	else if(htim->Instance == TIM1 &&
			htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		fp_state = FP_START;
		FP_Timeout_Stop();
		__set_PRIMASK(primask);
	}
	/*
	 * Called when the "downtime" for DHT sensor is up,
	 * and actual measurements should commence
	 * */
	else if(htim->Instance == TIM4 &&
			htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
		GPIO_InitTypeDef GPIO_InitStruct;
	    GPIO_InitStruct.Pin = DHT_IC_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
	    //pull up
		HAL_GPIO_WritePin(DHT_IC_GPIO_Port, DHT_IC_Pin, GPIO_PIN_SET);
		//set to input
	    HAL_GPIO_Init(DHT_IC_GPIO_Port, &GPIO_InitStruct);
	    dht_i = 0;
	    //enable IC
	    HAL_TIM_IC_Init(htim);
	    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
	    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 60000);
	    HAL_TIM_OC_Start_IT(htim, TIM_CHANNEL_2);
	}
	/*
	 * DHT Input Capture mode timeout.
	 * This timeout is reset during Input Capture automatically by setting the counter to 0.
	 * Except for the dht pointer, nobody is waiting for the event, so nothing has to be done.
	 */
	else if(htim->Instance == TIM4 &&
			htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_2);
		dht_i = 0;
	}
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef* hlptim)
{
	if(hlptim->Instance == LPTIM1)
		flags |= _FL_DISP_NEXT;
	else //LPTIM2
		flags |= _FL_BT_CONT;
}

uint32_t _dht_temp[43];

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance == TIM4)
	{
		uint16_t cmp = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_4);
		__HAL_TIM_SET_COUNTER(htim, 0);
		_dht_temp[dht_i++] = cmp;
		if(dht_i < 43)
		{
		      __HAL_TIM_ENABLE_IT(htim, TIM_IT_CC4);
		      TIM_CCxChannelCmd(htim->Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
		}
		else
		{
			memcpy(_dht11_timing_data, _dht_temp, 43*4);
			HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_2);
			flags |= _FL_DHT11_RCV;
			dht_i = 0;
		}
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
