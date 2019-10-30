/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "fifo_buffer.h"
#include "SPD_controller.h"

/* USER CODE BEGIN 0 */
	extern uint8_t tx8;
	extern uint8_t tx16[2];
	extern uint8_t tx24[3];
	extern uint8_t tx32[4];

	extern uint8_t rx8;
	extern uint8_t rx16[2];
	extern uint8_t rx24[3];
	extern uint8_t rx32[4];
	
	extern float floats[ADC_SUB_CHANNELS];
	extern FIFO_BUFF _fifos[FIFO_COUNT];
	
	extern SPI_HandleTypeDef hspi1;
	extern TIM_HandleTypeDef htim2;
	extern TIM_HandleTypeDef htim3;
	extern TIM_HandleTypeDef htim4;
	
	extern uint32_t cntr_pwr_on;
	extern uint32_t start_cntr_value;
	extern uint8_t query_det_pwr;
	
	extern PID_state PID_st;
	extern uint32_t cntr_TB3_PWR_ON;
	extern uint8_t TB3_PWR_EN;
	extern uint32_t TB3_PWR_CNTR;
	extern uint8_t TMR_mode;
	
	extern uint8_t TMR2OVF;
	extern uint32_t cntr_tim2_rst ;
	extern uint32_t cntr_tim2_val ;
	
	extern uint8_t TMR3OVF;
	
	extern uint8_t TMR4OVF ;
	extern uint32_t cntr_tim4_rst ;
	extern uint32_t cntr_tim4_val ;

	
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	HAL_NVIC_SystemReset();
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
	HAL_NVIC_SystemReset();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	HAL_NVIC_SystemReset();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
	HAL_NVIC_SystemReset();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

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
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void EXTI9_5_IRQHandler(void)
{

		/* USER CODE BEGIN EXTI9_5_IRQn 0 */

		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		//query_ADC_data_flag = 1;
		HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
		__HAL_GPIO_EXTI_CLEAR_FLAG(SPI1_RDY_ADC_Pin);
	
}

void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN */
		HAL_NVIC_DisableIRQ(TIM2_IRQn);	
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_UPDATE);
		//uint32_t counter = cntr_pwr_on;

		cntr_tim2_val--;
		if (cntr_tim2_val == 0)
		{
			TMR2OVF = 1;
			cntr_tim2_val = cntr_tim2_rst;
		}
		else
		{
			TMR2OVF = 0;
		}
			
	HAL_NVIC_EnableIRQ(TIM2_IRQn);	
  /* USER CODE END*/
}

void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN */
		HAL_NVIC_DisableIRQ(TIM3_IRQn);	
		__HAL_TIM_CLEAR_FLAG(&htim3, TIM_IT_UPDATE);
		TMR3OVF = 1;
		HAL_TIM_Base_Stop_IT(&htim3);

//		while( __HAL_TIM_GET_FLAG(&htim2,TIM_SR_UIF) == 0 );
//		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);

		//HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE END*/
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN */
	HAL_NVIC_DisableIRQ(TIM4_IRQn);	
	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_UPDATE);

	cntr_tim4_val--;
	if (cntr_tim4_val == 0)
	{
		TMR4OVF = 1;
//		cntr_tim4_val = cntr_tim4_rst;
//		HAL_NVIC_DisableIRQ(TIM4_IRQn);
//		HAL_TIM_Base_Stop_IT(&htim4);
//		
//		if(query_det_pwr == 0)
//			HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
//		else
//			HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
	}
	else
	{
		TMR4OVF = 0;
	}
			
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE END*/
}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
