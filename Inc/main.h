/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

//#define SPI4_CS1_Pin GPIO_PIN_3
//#define SPI4_CS1_GPIO_Port GPIOE
//#define SPI4_CS2_Pin GPIO_PIN_4
//#define SPI4_CS2_GPIO_Port GPIOE
//#define SPI4_PWDN_Pin GPIO_PIN_13
//#define SPI4_PWDN_GPIO_Port GPIOC
#define AIN0_TECI_Pin GPIO_PIN_0
#define AIN0_TECI_GPIO_Port GPIOA
#define AIN1_TECU_Pin GPIO_PIN_1
#define AIN1_TECU_GPIO_Port GPIOA
#define AIN2_THRM_Pin GPIO_PIN_2
#define AIN2_THRM_GPIO_Port GPIOA
#define AIN3_RF_PWR_Pin GPIO_PIN_3
#define AIN3_RF_PWR_GPIO_Port GPIOA
#define SHRED0_Pin GPIO_PIN_12
#define SHRED0_GPIO_Port GPIOB
#define SHRED1_Pin GPIO_PIN_13
#define SHRED1_GPIO_Port GPIOB
#define SHRED2_Pin GPIO_PIN_14
#define SHRED2_GPIO_Port GPIOB
#define SHRED3_Pin GPIO_PIN_15
#define SHRED3_GPIO_Port GPIOB
#define YELLOW_LED_Pin GPIO_PIN_8
#define YELLOW_LED_GPIO_Port GPIOD
#define RED_LED_Pin GPIO_PIN_9
#define RED_LED_GPIO_Port GPIOD
#define GREEN_LED_Pin GPIO_PIN_10
#define GREEN_LED_GPIO_Port GPIOD
#define I2C3_RST_Pin GPIO_PIN_7
#define I2C3_RST_GPIO_Port GPIOC
#define EEPROM_WC_Pin GPIO_PIN_8
#define EEPROM_WC_GPIO_Port GPIOC
#define SPI1_RDY_LVL_Pin GPIO_PIN_15
#define SPI1_RDY_LVL_GPIO_Port GPIOA
#define TP_SPI4_Pin GPIO_PIN_10
#define TP_SPI4_GPIO_Port GPIOC
#define TP_SPI1_Pin GPIO_PIN_11
#define TP_SPI1_GPIO_Port GPIOC
#define TP_I2C3_Pin GPIO_PIN_12
#define TP_I2C3_GPIO_Port GPIOC
#define TP_I2C1_Pin GPIO_PIN_0
#define TP_I2C1_GPIO_Port GPIOD
#define PG_TEC_Pin GPIO_PIN_1
#define PG_TEC_GPIO_Port GPIOD
#define PWR_TEC_CNTR_Pin GPIO_PIN_2
#define PWR_TEC_CNTR_GPIO_Port GPIOD
#define PWR_BASE_CNTR_Pin GPIO_PIN_3
#define PWR_BASE_CNTR_GPIO_Port GPIOD
#define PWR_DET_CNTR_Pin GPIO_PIN_4
#define PWR_DET_CNTR_GPIO_Port GPIOD
#define SPI1_CS_ADC_Pin GPIO_PIN_5
#define SPI1_CS_ADC_GPIO_Port GPIOD
#define SPI1_SYNC_ADC_Pin GPIO_PIN_6
#define SPI1_SYNC_ADC_GPIO_Port GPIOD
#define SPI1_RDY_ADC_Pin GPIO_PIN_7
#define SPI1_RDY_ADC_GPIO_Port GPIOD
#define SPI1_SCK_ADC_Pin GPIO_PIN_3
#define SPI1_SCK_ADC_GPIO_Port GPIOB
#define SPI1_MISO_ADC_Pin GPIO_PIN_4
#define SPI1_MISO_ADC_GPIO_Port GPIOB
#define SPI1_MOSI_ADC_Pin GPIO_PIN_5
#define SPI1_MOSI_ADC_GPIO_Port GPIOB
#define I2C1_RST_Pin GPIO_PIN_8
#define I2C1_RST_GPIO_Port GPIOB



/* USER CODE BEGIN Private defines */
#define SPI4_GT_ON_Pin GPIO_PIN_14
#define SPI4_GT_ON_GPIO_Port GPIOC

#define SPI4_RST_Pin GPIO_PIN_0
#define SPI4_RST_GPIO_Port GPIOE

#define SPI4_PWDN_Pin GPIO_PIN_13
#define SPI4_PWDN_GPIO_Port GPIOC

#define SPI4_CS0_Pin GPIO_PIN_1
#define SPI4_CS0_GPIO_Port GPIOE

#define SPI4_CS1_Pin GPIO_PIN_3
#define SPI4_CS1_GPIO_Port GPIOE

#define SPI4_CS2_Pin GPIO_PIN_4
#define SPI4_CS2_GPIO_Port GPIOE

#define SPI4_SCK_M_Pin GPIO_PIN_2
#define SPI4_SCK_M_GPIO_Port GPIOE

#define SPI4_MOSI_M_Pin GPIO_PIN_6
#define SPI4_MOSI_M_GPIO_Port GPIOE

#define SPI4_MISO_M_Pin GPIO_PIN_5
#define SPI4_MISO_M_GPIO_Port GPIOE

#pragma pack(push,1)

typedef struct
{
    unsigned char id; // report id
    unsigned char cmd; // command code
    unsigned char data[62]; // various data
} report_t;

#pragma pack(pop)

#define REPORTS_COUNT 5 

#define ADC_CHN0 0x0
#define ADC_CHN1 0x1
#define ADC_CHN2 0x2
#define ADC_CHN3 0x3

// структура для хранения результатов внутреннего АЦП
typedef struct
{
	float	CHN0_TB3_CURRENT;//сигнал с выхода микросхемы измерителя тока
	float	CHN1_TB3_VOLTAGE;//сигнал на выходе TEC драйвера
	float	CHN2_HEATSINK_THRM;//сигнал с датчика температуры радиатора ДОФ
	float	CHN3_RF_PWR;//сигнал с датчика РЧ мощности
} ADC_INT_RES;

typedef struct 
{
	unsigned char number;//valid num
	unsigned char TB3_pwr;// n%
	unsigned char DET_pwr;//ON/OFF
	float thrm_set;//target therm
	float thrm_lim;//limit therm
} thrm_profile;

typedef struct
{
	float dumping_temp;
	float leaving_temp;
	unsigned char EN_TIMER_OVERHEAT_FLAG;
	unsigned char EN_TEC1_FLAG;
	unsigned char EN_TEC3_FLAG;
	
	
} OVERHEAT_CNTRL;

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
