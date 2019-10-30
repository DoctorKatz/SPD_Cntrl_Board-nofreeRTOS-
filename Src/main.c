/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "stm32f4xx_hal_wwdg.h"

/* USER CODE BEGIN Includes */
#include "commands.h"
#include "fifo_buffer.h"
#include "ADC_AD7124.h"
#include "SPD_controller.h"
#include "EXT_EEPROM.h"
#include "math.h"

#define SOFTWARE_VERSION 0x06//версия прошивки контроллера
#define COMPLETE 0x7
#define	PWR_ON 1
#define	PWR_OFF 0

#define HV_code_min 0x6C90U

//#define THRM_PROF_TAB 6 //количество точек таблицы температурного профиля
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

IWDG_HandleTypeDef hiwdg;




/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t tx8 = 0;
uint8_t tx16[2] = { 0, 0 };
uint8_t tx24[3] = { 0, 0, 0 };
uint8_t tx32[4] = { 0, 0, 0, 0 };

uint8_t rx8 = 0;
uint8_t rx16[2] = { 0, 0 };
uint8_t rx24[3] = { 0, 0, 0 };
uint8_t rx32[4] = { 0, 0, 0, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);
static void MX_IWDG_Init_Overheat(void);


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MX_CRC_Init(void);
//void TMR2_Init(void);

uint8_t control_version(void);
uint8_t POWER_DET_CNTR(uint8_t power_state);
//void POWER_TEC_TB1_CNTR(uint8_t power_state, uint8_t power_value);
void POWER_TEC_TB1_CNTR(uint8_t power_state);
void POWER_TEC_TB3_CNTR(uint8_t power_state);
uint8_t pwr_control(uint8_t index);
uint8_t SDADC_conv (uint8_t chn, uint32_t *readed_data );
uint8_t conv_resp = 0;
uint8_t SDADC_chn = 0;
uint32_t readed_data = 0;
uint32_t SDADC_cntr_fail_read = 0;//счетчик пропущенных отсчетов внешнего АЦП

float calc_thrm_heatsink(float voltage);

uint8_t select_chn_ADC(uint8_t channel);

extern uint8_t VarRes_write(uint8_t DATA);
extern uint8_t VarRes_read(uint8_t *DATA);
extern uint8_t TEC_DRIVER_INIT(void);
extern uint8_t tec_volt_cntr(float percent);
float calc_response_PID(float input, float input_previous);//input - температура
inline float calc_rf_pwr(float inp);

//uint8_t calc_response_power_TB1(float Tset, float Tcurr);
uint8_t pwr_set_TB3(float thrm);
//eeprom procedures
extern uint8_t eeprom_test(void);
extern uint8_t eeprom_write (uint16_t start_addr, uint8_t *data);
extern uint8_t eeprom_read (uint16_t start_addr, uint8_t *data);
extern uint8_t copy_to_eeprom(uint16_t  addr_dest, void * ptr_source, uint16_t size_source);
extern uint8_t copy_from_eeprom(uint16_t  addr_source, void * ptr_dest, uint16_t size_source);

uint32_t takeData(uint8_t index, float* dst);
float takeTherm(float value_pt1000);
uint16_t inc_tail(uint16_t tail_val);
uint16_t inc_head(uint16_t head_val);
float abs_value(uint8_t *sign, float value);
uint8_t thrm_capture(float thrm_inp, float thrm_trg, float thrm_lim);
uint8_t thrm_inrange(float thrm_inp, float thrm_trg, float thrm_lim);
void tim3_delay(void);

extern void ADC_RG_INIT(void);
extern uint8_t ADC_SETUP_0(void);
extern uint8_t ADC_CALIBR(uint8_t meas_chn);
extern uint8_t ADC_change_filter_length(uint16_t new_length);
extern uint32_t ADC_read_DATA(uint8_t *channel, uint8_t *status);

extern void PID_init(PID_regs *, PID_state *);
extern void SPD_init(SPD_regs *);
extern void HV_source_init(HV_src *);
extern void DDS_init(DDS_regs *DDS);
extern void DET_init(DET_control *DET);
extern void sleep_DDS(void);
extern void wakeup_DDS(void);
extern uint8_t DDS_stb_write(uint16_t inp_code);
extern uint8_t DDS_stb_read(uint16_t *outp_code);


extern uint8_t calibrate_TEC_DRIVER(void);
extern void VarRes_reset(void);

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef	*pdev, 
																					uint8_t *report,
																					uint16_t len);

//DAC HV_source
extern uint8_t DAC_EXT_write(uint16_t data);//DAC8411 external DAC 16 bit
extern uint8_t DAC_INT_write(uint8_t data);//MAX1932 internal DAC 8 bit
extern void DAC_EXT_init(uint16_t data);
extern uint8_t DAC_EXT_calib(void);
//extern uint8_t HV_code_to_value (uint16_t inp_code);
//extern uint8_t HV_value_to_code (float inp_volt_val);
extern uint8_t HV_calib_res (float inp_adc_value, float *outp_value);
extern uint8_t HV_value_set_outp (float *inp_value);
extern uint8_t HV_code_set_outp (uint16_t *inp_code);
//DDS
extern uint8_t write_DDS(uint16_t addr, uint8_t *data, uint8_t num_bytes);
extern uint8_t read_DDS(uint16_t addr, uint8_t *data, uint8_t num_bytes);
extern uint8_t init_DDS(void);
extern void DAC_EXT_init(uint16_t data);
volatile uint8_t Detector_number = 0;

uint32_t frameCounter;
uint8_t USB_TX_Buffer[64];
report_t reports[2]; 
volatile uint8_t received = 0;
volatile uint8_t lockedReportIndex = 1;
volatile uint8_t newReportIndex = 1;

ADC_ChannelConfTypeDef adc_config;

FIFO_BUFF _fifos[FIFO_COUNT];
uint32_t _result = 0;
float floats[ADC_SUB_CHANNELS];
float floats_old[ADC_SUB_CHANNELS];
float calc_mean_value (float sample);

const uint8_t max_code_res = 255;

//переменные фильтра скользящего среднего
const uint16_t smooth_width = (uint16_t)SIZE - 1 ;

FIFO_BUFF samples_sqn;
uint32_t samples_cntr = 0;

//коэффициенты пересчета сопротивления в температуру
const float vect_coeff [4] = {-2.4351e2, 2.2823e-1, 1.8217e-5 , -2.9379e-9};

//коэффициенты линейной характеристики пересчета мощности
const float vect_rfpwr_coeff[2] = { 1.0/0.0213 , -49.0 } ;

uint32_t emerg_cntr = 0;//счетчик циклов, проведённых при повышенной темепратуре

uint8_t PWR_RES_FLAG = 0;
uint8_t PWR_RES_started = 0;
uint8_t PWR_AUTO_RES_en = 1;

//флаги запроса на сохранение настроек 
uint8_t EEPROM_save_T_set_flag = 0;
uint8_t EEPROM_save_HV_trg_flag = 0;
uint8_t EEPROM_save_DDS_STB_flag = 0;
uint8_t EEPROM_save_SPD_serial_flag = 0;
uint8_t EEPROM_save_SPD_name_flag = 0;
uint8_t EEPROM_save_SPD_number = 0;
uint8_t EEPROM_save_SPD_Overheat_temp = 0;
uint8_t EEPROM_save_EN_TIMER_OVERHEAT_FLAG_flag = 0;		
uint8_t	EEPROM_save_leaving_temp_flag	= 0;
uint8_t EEPROM_save_dumping_temp_flag = 0;

//thrm_capture_en - соотв. функции thrm_capture
uint8_t thrm_capture_en = 0;//тумблер разрешения работы функции capture

//thrm_inrange_flag - соотв. функции thrm_inrange
uint8_t thrm_inrange_flag = 0;//флаг нахождения в пределах диапазона температур

uint8_t thrm_capture_trig = 0;//флаг "захвата" темп/й области..
//..т.е. флаг выставляется при нахождении в пределах области в теч/и времени cntr_tmr2_rst
//uint8_t thrm_capture_trig_cntr = 0 ;// счетчик стабилизации в темп/й области

uint8_t query_det_pwr = 0;//флаг включения сигнальной платы (вкл/выкл)
uint8_t TimerEn = 0;//разрешение включения таймера
uint8_t IsTimerEn = 0;//переменная триггер для хранения предыдущего значения TimerEn

uint8_t TMR2OVF = 0;
uint8_t TMR3OVF = 0;//таймер для выдержки линий DDS & HV_DAC
uint8_t TMR4OVF = 0;
//uint8_t TMR5OVF = 0;

uint32_t cntr_tim2_rst = 30; //начальное значение таймера 2
uint32_t cntr_tim2_val = 0;//текущее значение таймера 2
uint8_t	tim2_started = 0;//флаг запуска таймера 2
uint8_t tmr2_en = 1; //тумблер таймера 2

//таймер защитного интервала включения питания
uint32_t cntr_tim4_rst = 3;//секунды
uint32_t cntr_tim4_val = 0;
uint8_t tmr4_started = 0;



volatile uint8_t Detector_number_wr;


//const uint32_t cntr_tim5_rst = 30;
//uint32_t cntr_tim5_val = 0;
//uint8_t tim5_en_flag = 0;




PID_regs PID;
PID_state PID_st;
HV_src HV;
DDS_regs DDS;
DET_control DET;
SPD_regs SPD;

uint8_t sign = 0 ;

//параметры рабочей точки
float T_target = -45.0 ;//параметр центра темп/й области
const float T_lim = 0.05 ;//параметр границ стационарной темп/й области
//const float T_lim_coarse = 1.5 ;//параметр границ ближней темп/й области 
//const float T_lim_pwr = 0.5;//границы включения питания детектора
uint8_t tec_pwr_lim = 1;//предельное значение мощности в %
const float T_ht_trg = 30.0 ;//порог температуры радиатора для включения детектора
const float T_ht_lim = 3.5 ;//границы изменения температуры
uint8_t thrm_zone = 0;//номер температурной зоны
uint8_t temp_status = 0; //статус режима работы: 0 - не выешл на температурный режим, напряжения на фотодиоде нет
												 //1 - вышел на режим, напряжение на фотодиоде подано; 2 - ушел с режима напряжение на фотодиоде подано

float T_temp_target = 0.0 ;
float	T_temp_lim = 0.0 ;
float T_temp = 0.0;

//коэффициенты пересчета бинарного кода в физические величины
extern line_coeff ch_thrm;
extern line_coeff ch_curr;
extern line_coeff ch_hv;

//регистры внутренней калибровки АЦП
extern uint32_t AD7124_OFFSET0;
extern uint32_t AD7124_OFFSET1;
extern uint32_t AD7124_OFFSET2;
extern uint32_t AD7124_GAIN0;
extern uint32_t AD7124_GAIN1;
extern uint32_t AD7124_GAIN2;
extern uint32_t AD7124_CALIB_SET[6];

//коэфф. калибровки HV
extern struct
{
	double C0;
	double C1;
	double B0;
	double B1;
	double alpha0;
	double alpha1;
	double beta0;
} HV_coeff;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	ADC_INT_RES ADC_res;
	OVERHEAT_CNTRL overheat_achtung;
	uint8_t ADC_chn_sel = 0;

	uint8_t sta = 0;
	uint16_t addr_src = 0;
	uint16_t addr_dest = 0;
	uint16_t size_src = 0;
	void *ptr_dst = NULL;
	void *ptr_src = NULL;
	
	uint8_t str_SPD_name[62];

	uint8_t tec_drv_resp = 0;
	uint32_t tec_drv_fail_cntr = 10;
	
	uint8_t eeprom_data_rd;
	uint8_t buffer_rd[4];
	uint8_t *ptr_dest = buffer_rd;
	uint16_t addr_source ;
	int rdd_result ;
	uint32_t cntr_sample_in_range = 0;//количество точек попавших в нужную область

	
	FIFO_BUFF * p = _fifos;
	for (uint32_t i = 0; i < FIFO_COUNT; i++) {
			buffer_init(p);
			Clear(p++);
	}
	
	p = &samples_sqn;
	buffer_init(p);
	Clear(p);
	samples_cntr = 0;
	

	USB_TX_Buffer[0] = 1;
	USB_TX_Buffer[1] = 0;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	
	//контроль версии ПО
	//if(control_version()==0)
	//	Error_Handler();
		
  MX_ADC1_Init();
  ////MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  //////MX_SPI4_Init();//!!!software realisation
  MX_SPI1_Init();
  MX_I2C3_Init();//for synchrodetector control
	MX_CRC_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	//MX_IWDG_Init();//инициализация IWDG непосредственно перед рабочим циклом
	
	HAL_NVIC_DisableIRQ(TIM2_IRQn);	
	HAL_NVIC_DisableIRQ(TIM3_IRQn);	
	HAL_NVIC_DisableIRQ(TIM4_IRQn);	
	
  /* USER CODE BEGIN 2 */
		
	//выключение TEC элементов
	POWER_TEC_TB1_CNTR(PWR_OFF);
	POWER_TEC_TB3_CNTR(PWR_OFF);
	
	//выключение питания детектора
	HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
	
	//светодиоды
	HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);

//инициализация внутренних переменных контроллера (MCU)
	PID_init(&PID, &PID_st);
	SPD_init(&SPD);
	HV_source_init(&HV);
	DDS_init(&DDS);
	DET_init(&DET);
	
	//проверка связи с интерфейсом TEC-драйвера
	tec_drv_resp = 0;
	while( tec_drv_resp == 0 )
	{
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
		VarRes_reset();
		HAL_Delay(10);
		tec_drv_resp = TEC_DRIVER_INIT();
		if (tec_drv_resp == 0)
			tec_drv_fail_cntr--;
		else
		{
			HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
			SPD.SPD_status_reg &= ~SPD_TEC_driver_err_flag ;
			break;
		}
		
		if (tec_drv_fail_cntr == 0)
		{
			HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
			SPD.SPD_status_reg |= SPD_TEC_driver_err_flag ;
			//Error_Handler();
		}
	}
		

	
	//HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
	//DET.IsPowerOn = 1;
	//SPD.SPD_status_reg |= SPD_DET_pwr_flag;
	//HAL_Delay(100);
	DAC_EXT_init(HV_code_min);//установка напряжения в минимум: +5В
	init_DDS();
	HAL_Delay(1000);
	
	// *************************************************
	//инициализация АЦП
		//настройка регистров АЦП
		ADC_RG_INIT();

		//проверка наличия готовой калибровки
		addr_dest = BASE_DEV_DESC + OFS_TAB_PT_VALID;
		uint8_t state_PT = 0;
		sta =	eeprom_read(addr_dest, &state_PT);
		if ( (sta==1) && (state_PT!=0) && (state_PT!=0xFF) )
		{
			
			//если есть калибровка, то считываем данные регистров OFFSET и GAIN
			addr_src = BASE_DEV_DESC + BASE_PT_SET;
			ptr_dst = AD7124_CALIB_SET;
			size_src = sizeof(AD7124_CALIB_SET);
			sta = copy_from_eeprom(addr_src, ptr_dst, size_src);
			if (sta!=1)
			{
				goto AD7124_CALIB;//если чтение данных неудачно, то выполняем калибровку
			}
			//если четние было успешным, то настраиваем АЦП значениями из памяти
			SPD.SPD_status_reg |= SPD_SDADC_calib_flag ;
			//переносим данные из массива в переменные
			AD7124_GAIN0 = AD7124_CALIB_SET[0];
			AD7124_GAIN1 = AD7124_CALIB_SET[1];
			AD7124_GAIN2 = AD7124_CALIB_SET[2];
			AD7124_OFFSET0 = AD7124_CALIB_SET[3];
			AD7124_OFFSET1 = AD7124_CALIB_SET[4];
			AD7124_OFFSET2 = AD7124_CALIB_SET[5];
			//настройка АЦП значениями из регистров
			ADC_SETUP_0();
		}
	else
	{
	AD7124_CALIB:		
		//сбрасываем флаг калибровки
		SPD.SPD_status_reg &= ~SPD_SDADC_calib_flag ;
		state_PT = 0;
		addr_dest = BASE_DEV_DESC + OFS_TAB_PT_VALID;
		eeprom_write(addr_dest, &state_PT);//записываем актуальность калибровки
		//выключение питания детектора
		HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
		
		//настройка АЦП значениями из регистров
		ADC_SETUP_0();
		//калибровка измерительных каналов АЦП
		ADC_CALIBR(0);
		ADC_CALIBR(1);
		ADC_CALIBR(2);

		//переносим данные в массив
		AD7124_CALIB_SET[0] = AD7124_GAIN0;
		AD7124_CALIB_SET[1] = AD7124_GAIN1;
		AD7124_CALIB_SET[2] = AD7124_GAIN2;
		AD7124_CALIB_SET[3] = AD7124_OFFSET0;
		AD7124_CALIB_SET[4] = AD7124_OFFSET1;
		AD7124_CALIB_SET[5] = AD7124_OFFSET2;
					
		//сохранение в EEPROM
		
		addr_dest = BASE_DEV_DESC + BASE_PT_SET;
		ptr_src = AD7124_CALIB_SET;
		size_src = sizeof(AD7124_CALIB_SET);
		sta = copy_to_eeprom(addr_dest, ptr_src, size_src);//записываем значения вычисленных коэффициентов
		state_PT = 0;
		if (sta==1)
		{
			state_PT = 1;
			SPD.SPD_status_reg |= SPD_SDADC_calib_flag ;
		}
		//записываем флаг калибровки
		addr_dest = BASE_DEV_DESC + OFS_TAB_PT_VALID;
		eeprom_write(addr_dest, &state_PT);//записываем актуальность калибровки
	}
	//завершение инициализации АЦП
	// ************************************************
	DAC_EXT_init(HV_code_min);
	HAL_Delay(1000);
	addr_dest = BASE_DEV_DESC + OFS_TAB_HV_VALID;
	HV.state = 0;
	sta =	eeprom_read(addr_dest, &HV.state );
	if ( (sta==1) && (HV.state != 0) && (HV.state != 0xFF) )
	{
		addr_src = BASE_DEV_DESC + BASE_HV_DAC_SET;
		ptr_dst = &HV_coeff;
		size_src = sizeof(HV_coeff);
		for(uint16_t rr = 0; rr < 3 ; rr++)
		{
			sta = copy_from_eeprom(addr_src, ptr_dst, size_src);
			if(sta == 1)
			{
				HV.IsErrors = 0;
				SPD.SPD_status_reg |= SPD_HV_drv_calib_flag ;
				break;
			}
			else
			{
				HV.IsErrors = 1;
				SPD.SPD_status_reg &= ~SPD_HV_drv_calib_flag ;
			}
		}
		
		if(HV.IsErrors == 0)
		{
			//вычисляем код
			//HV.code = (uint16_t)(( HV.HV_target -  HV_coeff.B0 ) / HV_coeff.B1) ;
			HV.code = (uint16_t)(( 5.0 -  HV_coeff.B0 ) / HV_coeff.B1) ;
		}
		else
		{
			HV.code = HV_code_min;//5.0V
		}
		DAC_EXT_write(HV.code);	
		HV.HV_value = (float)( HV_coeff.B0 + HV_coeff.B1 * (double)HV.code ) ;
		HV.IsCalibrate = 1;
		HV.IsErrors = 0;
		//HAL_Delay(1000);

		//чтение HV_target из EEPROM
		for (uint16_t ii = 0; ii < 2; ii++)
		{
				addr_source = BASE_HV_SET + ii;
				eeprom_read(addr_source, &eeprom_data_rd);
				*(uint8_t*)ptr_dest = eeprom_data_rd;
				ptr_dest++;
		}
		rdd_result = 0;
		rdd_result = (buffer_rd[1] << 8)|buffer_rd[0];
		uint16_t rdd_code = (uint16_t)(rdd_result);
		HV.HV_target = (float)( HV_coeff.B0 + HV_coeff.B1 * (double)rdd_code ) ;
	}
	else
	{
		SPD.SPD_status_reg &= ~SPD_HV_drv_calib_flag ;
		HV.state = 0;
		addr_dest = BASE_DEV_DESC + OFS_TAB_HV_VALID;
		eeprom_write(addr_dest, &HV.state); //записываем актуальность калибровки
		//калибровка внешнего ЦАП HV блока
		sta = DAC_EXT_calib();
		if (sta == 0)
		{
			HV.IsCalibrate = 0;
			HV.IsErrors = 1;
			
		}
		else
		{
			HV.IsCalibrate = 1;
			HV.IsErrors = 0;
			SPD.SPD_status_reg |= SPD_HV_drv_calib_flag ;		
			addr_src = BASE_DEV_DESC + BASE_HV_DAC_SET;
			ptr_dst = &HV_coeff;
			size_src = sizeof(HV_coeff);
			sta = copy_to_eeprom(addr_src, ptr_dst, size_src);
			if(sta == 1)
				HV.state = 1;
			else
				HV.state = 0;
			
			addr_dest = BASE_DEV_DESC + OFS_TAB_HV_VALID;
			eeprom_write(addr_dest, &HV.state); //записываем актуальность калибровки
			
		}
	}
	
	HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
	DET.IsPowerOn = 0;
	//*************************************************

	//настройки TEC-драйвера
	//чтение флага наличия калибровки
	addr_dest = BASE_DEV_DESC + OFS_TAB_PID_VALID;
	uint8_t state_PID = (PID_st.state & 0x01);
	sta =	eeprom_read(addr_dest, &state_PID);
	if ( (sta==1) && (state_PID!=0) && (state_PID!=0xFF) )
	{
		//если калиброка есть, то считываем её из памяти
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
		
		addr_src = BASE_DEV_DESC + BASE_PID_SET;
		ptr_dst = &PID;
		size_src = sizeof(PID);
		sta = copy_from_eeprom(addr_src, ptr_dst, size_src);
		PID_st.IsErrors = 0;
		if (sta!=1) PID_st.IsErrors = 1;
		
		SPD.SPD_status_reg |= SPD_TEC_drv_calib_flag ;
		PID_st.tg_calc = (PID.max_dc_volt - PID.min_dc_volt) / (int)((uint8_t)PID.index_max_volt + (-1) * (uint8_t)PID.index_min_volt );
		PID_st.ofs_calc = PID.max_dc_volt - PID_st.tg_calc * (int)PID.index_max_volt;
		PID_st.tg_calc = (float)1.0 / PID_st.tg_calc;
		
							
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		//если сохранённой калибровки нет или CRC-неверно, то проводим калибровку
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
		state_PID = 0;
		addr_dest = BASE_DEV_DESC + OFS_TAB_PID_VALID;
		eeprom_write(addr_dest, &state_PID);//сбрасываем актуальность калибровки
		PID_st.IsCalibrate = 0;
		
		POWER_TEC_TB1_CNTR(PWR_ON);
		uint8_t calib_st = 0;
		calib_st = calibrate_TEC_DRIVER();
		if (calib_st == 1)
		{
			//сохранение в EEPROM
			addr_dest = BASE_DEV_DESC + BASE_PID_SET;
			ptr_src = &PID;
			size_src = sizeof(PID);
			PID_st.IsCalibrate = 0;
			state_PID = 0;
			sta = 0;
			sta = copy_to_eeprom(addr_dest, ptr_src, size_src);//записываем значения вычисленных коэффициентов
			if (sta==1)
			{
					SPD.SPD_status_reg |= SPD_TEC_drv_calib_flag ;
					state_PID = 0x1;
					PID_st.IsCalibrate = 1;
			}
			addr_dest = BASE_DEV_DESC + OFS_TAB_PID_VALID;
			eeprom_write(addr_dest, &state_PID);//записываем актуальность калибровки

			HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
		}
		else
		{
			Error_Handler();
		}
	}
	
	T_target = PID.T_set ;//параметр центра темп/й области
	
	PID_st.IsCalibrate = 1;
	PID_st.IsPowerOn = 1;
	tec_volt_cntr(0.0);		
	POWER_TEC_TB3_CNTR(PWR_ON);
	//завершение инициализации TEC драйвера
	// *************************************************
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);
	volatile GetData_TX_t* pData = (GetData_TX_t*)(USB_TX_Buffer + 2);
	
	uint16_t adc_res_uint = 0;
	float adc_res_float = 0;
	float response;
	float PT_resist = 1000.0;
	float tVD = 0.0;
	float tVD_old = 0;
	float tVD_round = 0;	// переменная округленной температуры фотодиода
	uint8_t sign = 0;
	
	HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET) ;
		
	//чтение Tset из EEPROM
	float rdd_thrm = 0.0;
	float rdd_thrm_round = 0;	// переменная для округленной температуры - уставки;

	ptr_dest = buffer_rd;
	addr_source = BASE_T_SET;
	for (uint16_t ii = 0; ii < sizeof(buffer_rd); ii++)
	{
			addr_source = BASE_T_SET + ii;
			eeprom_read(addr_source, &eeprom_data_rd);
			*(uint8_t*)ptr_dest = eeprom_data_rd;
			ptr_dest++;
	}
	rdd_result = (buffer_rd[3] << 24)|(buffer_rd[2] << 16)|(buffer_rd[1] << 8)|buffer_rd[0];
	//rdd_thrm = (float)(rdd_result*(-1)/1000);
	rdd_thrm = *(float*)&rdd_result;

		
	float thrm_t0 = (float)0.0 ;
	uint8_t num_average = 4;
	//проверка начального значения температуры
	for(uint16_t num_samples = 0 ; num_samples < num_average ; num_samples++)
	{
		do
		{
			conv_resp = 0;
			for(SDADC_chn = 0 ; SDADC_chn < 3 ; SDADC_chn++)
				conv_resp += SDADC_conv (SDADC_chn, &readed_data );
		} while( conv_resp < 3 ) ;
		
		PT_resist = ADC_REF_RES * floats[0]/floats[1];//сопротивление термистора
		thrm_t0 += takeTherm(PT_resist);//вычисленная температура
		
	}
	thrm_t0 *= (float)1.0 / (float)num_average;

	//управление подачей питания на TEC-элементы и детектор
	if ( (rdd_thrm>=(float)(-90.0))&&(rdd_thrm<= (float)0.0) )
	{
		PID.T_set = rdd_thrm;
		T_target = PID.T_set;
		T_temp_target = T_target ;
		T_temp_lim = T_lim;
	}
		//определение мощности при старте
		pwr_set_TB3(thrm_t0);
		POWER_TEC_TB1_CNTR(PWR_ON);
		POWER_TEC_TB3_CNTR(PWR_ON);
		HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
		DET.IsPowerOn = 0;
	
	
	//чтение DDS_STB из EEPROM
	ptr_dest = buffer_rd ;
	for (uint16_t ii = 0; ii < 2; ii++)
	{
			addr_source = BASE_STB_SET + ii;
			eeprom_read(addr_source, &eeprom_data_rd);
			*(uint8_t*)ptr_dest = eeprom_data_rd;
			ptr_dest++;
	}
	rdd_result = 0;
	rdd_result = (buffer_rd[1] << 8)|buffer_rd[0];
	DDS.STB = (uint16_t)(rdd_result);
	//DDS_stb_write(DDS.STB);
	
	//чтение номера детектора из EEPROM
	uint8_t temporary_uint8_t = 0;
	eeprom_read(BASE_SPD_NUMBER, &temporary_uint8_t);
	Detector_number = temporary_uint8_t;
	
	//чтение температуры перегрева из EEPROM
	/*float temporary_float = 0;
	addr_src = BASE_SPD_OVERHEAT_TEMP;
	ptr_dst = &temporary_float;
	size_src = sizeof(BASE_SPD_OVERHEAT_TEMP);
	sta = copy_from_eeprom(addr_src, ptr_dst, size_src);
	SPD.T_emerg = *((float*)ptr_dst);*/
	
	//чтение параметров перегрева из EEPROM
	float temporary_float = 0;
	addr_src = BASE_DUMPING_TEMP;
	ptr_dst = &temporary_float;
	size_src = sizeof(BASE_DUMPING_TEMP);
	sta = copy_from_eeprom (addr_src, ptr_dst, size_src);
	overheat_achtung.dumping_temp = *((float*)ptr_dst);
	
	/*addr_src = BASE_EN_TIMER_OVERHEAT_FLAG;
	ptr_dst = &temporary_float;
	size_src = sizeof(BASE_EN_TIMER_OVERHEAT_FLAG);
	sta = copy_from_eeprom (addr_src, ptr_dst, size_src);
	overheat_achtung.EN_TIMER_OVERHEAT_FLAG = *((float*)ptr_dst);*/
	
	/*eeprom_read (BASE_EN_TIMER_OVERHEAT_FLAG, &temporary_uint8_t);
	overheat_achtung.EN_TIMER_OVERHEAT_FLAG = temporary_uint8_t;*/
	
/*
	eeprom_read (BASE_EN_TEC1_FLAG, &temporary_uint8_t);
	overheat_achtung.EN_TEC1_FLAG = &temporary_uint8_t;
	
	eeprom_read (BASE_EN_TEC3_FLAG, &temporary_uint8_t);
	overheat_achtung.EN_TEC3_FLAG = &temporary_uint8_t*/
	
	addr_src = BASE_LEAVING_TEMP;
	ptr_dst = &temporary_float;
	size_src = sizeof(BASE_LEAVING_TEMP);
	sta = copy_from_eeprom (addr_src, ptr_dst, size_src);
	overheat_achtung.leaving_temp = *((float*)ptr_dst);
	
	
	
	MX_USB_DEVICE_Init();
	HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

	PID_regs PID_to_write;
	cntr_sample_in_range = 0;	
	
	//MX_IWDG_Init();
	//__HAL_IWDG_START(&hiwdg);
	
	while (1)//основной цикл
  {

		//считывание отсчетов внешнего АЦП
		do 
		{
			conv_resp = 0;
			for(SDADC_chn = 0 ; SDADC_chn < 3 ; SDADC_chn++)
				conv_resp += SDADC_conv (SDADC_chn, &readed_data );
			
			if(conv_resp == 3)
				break;
		} while (conv_resp < 3) ;
				
		//считывание отсчетов внутреннего АЦП
		if (select_chn_ADC(ADC_chn_sel))
		{
			HAL_ADC_Start(&hadc1);
			hadc1.Instance->CR2 |= 0x40000000;
			if(HAL_ADC_PollForConversion(&hadc1, 10000) == HAL_OK)
			{
					adc_res_uint = HAL_ADC_GetValue(&hadc1);
					HAL_ADC_Stop(&hadc1);
					adc_res_uint &= 0x0FFF;
					adc_res_float = (float)(adc_res_uint * (2.5/4096));//встроенный АЦП
				switch(ADC_chn_sel)
				{
					case 0:
					{
						ADC_res.CHN0_TB3_CURRENT = adc_res_float * PID.factor_TEC_CURRENT;	
						break;
					}
					case 1:
					{
						ADC_res.CHN1_TB3_VOLTAGE = adc_res_float * PID.factor_TEC_VOLTAGE ;	
						break;
					}
					case 2:
					{
						ADC_res.CHN2_HEATSINK_THRM = calc_thrm_heatsink(adc_res_float) ;	
						break;
					}
					case 3:
					{
						//ADC_res.CHN3_RF_PWR = adc_res_float ;
						ADC_res.CHN3_RF_PWR = calc_rf_pwr(adc_res_float) ;
						break;
					}
				}
				
			}
			HAL_ADC_Stop(&hadc1);
			ADC_chn_sel++;
			if (ADC_chn_sel > 3) ADC_chn_sel = 0;
		}
				
		//проверка на адекватность показаний внешнего АЦП
		if( (floats[1] == 0) || ( abs_value(&sign, floats[1]) >= (float)0.15 ) )//100uA х 1kOhm + err
		{//если возникла ошибка при чтении отсчета АЦП
			conv_resp = 0;
			SDADC_chn = 1;
			uint8_t cntr_adc_fail_conv = 5 ;
			while(conv_resp == 0)
			{
				conv_resp = SDADC_conv (SDADC_chn, &readed_data );
				if(conv_resp == 1)
					break;				
				else
				{
					cntr_adc_fail_conv--;
					if (cntr_adc_fail_conv == 0)
					{
						//reset ADC or SPI interface
						ADC_RG_INIT();
						ADC_SETUP_0();
						
						for(uint8_t ii = 0; ii < ADC_SUB_CHANNELS ; ii++)
							floats[ii] = floats_old[ii];
						
						SDADC_cntr_fail_read++;
						if(SDADC_cntr_fail_read > 1000)
						{
							//Error_Handler();
							SPD.SPD_status_reg |= SPD_SDADC_err_flag ;
							SPD.SPD_status_reg |= SPD_PT_err_flag ;
						}
						break;
					}
				}
			}
		}
		else
		{//если все отсчеты - действительные
			//floats[0] - напряжение PT
			//floats[1] - ток PT
			SDADC_cntr_fail_read = 0;
			SPD.SPD_status_reg &= ~SPD_SDADC_err_flag ;
			SPD.SPD_status_reg &= ~SPD_PT_err_flag ;
			
			for(uint8_t ii = 0; ii < ADC_SUB_CHANNELS ; ii++)
				floats_old[ii] = floats[ii];
			
			PT_resist = ADC_REF_RES * floats[0]/floats[1];//сопротивление термистора
			tVD = takeTherm(PT_resist);//вычисленная температура
			tVD_round = round(tVD);
			
			//ограничиваем маскимальный предел мощности на ТВ3
			pwr_set_TB3(tVD);
			//выставляем мощность, упирающийся в максимальное значение
			tec_volt_cntr(100);	
				
				
			if(thrm_capture_trig == 0)//если срабатывания триггера по таймеру не было
			{
				
				//селектор температуры
				//if(thrm_zone == 1)
				//	T_temp = ADC_res.CHN2_HEATSINK_THRM ;
				//else
					T_temp = tVD ;
				
					
				//запуск таймера стационарного теплового режима
				//thrm_capture_trig = thrm_capture(tVD, T_target, T_lim) ;
				thrm_capture_trig = thrm_capture(T_temp, T_temp_target, T_temp_lim) ;
				if( thrm_capture_trig == 1  )
				{
					switch(thrm_zone)			
					{
						case 0://стац.тепл.режим диода с выключенным детектором
						{
							thrm_zone = 1;
							//T_temp_target = T_ht_trg ; 
							//T_temp_lim = T_ht_lim;
							T_temp_target = T_target ; 
							T_temp_lim = T_lim;
							tmr2_en = 1 ;
							thrm_capture_trig = 0;
							cntr_tim2_rst = 120;
							SPD.SPD_status_reg |= SPD_warm_up_flag ;
							break;
						}
						
						case 1://стац.тепл.режим радиатора
						{
							thrm_zone = 2;
							T_temp_target = T_target ; 
							T_temp_lim = T_lim;
							tmr2_en = 1 ;
							thrm_capture_trig = 0;
							cntr_tim2_rst = 30;
							SPD.SPD_status_reg |= SPD_warm_up_flag ;
							if( (PWR_RES_FLAG == 0) && (PWR_AUTO_RES_en == 1) )
							{
								PWR_RES_FLAG = 1;//выставляем запрос на автом. сброс питания
								PWR_AUTO_RES_en = 0;
								//HV_value_set_outp(&HV.HV_target);
								//sleep_DDS();
								//HAL_Delay(10);
								//wakeup_DDS();
								//init_DDS();
							}
							break;
						}
						
						case 2://стац.тепл.режим после сброса питания детектора
						{
							SPD.SPD_status_reg &= ~SPD_warm_up_flag ;
							//HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
							break;
						}
						case 3:
						{
							HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
							DET.IsPowerOn = 0 ;
							SPD.SPD_status_reg &= ~SPD_DET_pwr_flag ;
							tec_pwr_lim = 50;
							SPD.SPD_status_reg |= SPD_emerg_stop_flag ;
						break;					
						}
						
						default:
							break;
					}
				}
				else//если захвата не было
				{
					tmr2_en = 1;//разрешаем таймер для последующего запуска
					HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
					SPD.SPD_status_reg &= ~SPD_ready_flag ;
				}
				
				
			}
			else//если был зазват
			{
				switch(thrm_zone)
				{
					case 0:
					{
						HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
						SPD.SPD_status_reg &= ~SPD_ready_flag ;
						break;
					}
					case 1:
					{
						HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
						SPD.SPD_status_reg &= ~SPD_ready_flag ;
						break;
					}
					case 2:
					{
						if( thrm_inrange(tVD, T_target, T_lim) )
						{
							HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
							SPD.SPD_status_reg |= SPD_ready_flag ;
						}
						else
						{
							HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
							SPD.SPD_status_reg &= ~SPD_ready_flag ;
						}
						break;
					}
					case 3:
					{
						HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
						DET.IsPowerOn = 0 ;
						SPD.SPD_status_reg &= ~SPD_DET_pwr_flag ;
						tec_pwr_lim = 50;
						SPD.SPD_status_reg |= SPD_emerg_stop_flag ;
						break;					
					}
					default:
					{	
						break;
					}
				}
			}
		
			
			//Put( *(uint32_t*)&tVD, &samples_sqn);
			//if (GetPendingSize(&samples_sqn) >= smooth_width)
			//{
			//	Get( (uint32_t*)&tVD_old , &samples_sqn);
			//}
			//else
					tVD_old = tVD;
				
			response = calc_response_PID(tVD, tVD_old);
			response *= (float)100.0;
			tec_volt_cntr(response);
				
				
		
		}
		
		if( PWR_RES_FLAG == 1 )
		{	
//			//если попали в область захвата..
//			//..то выполняем сброс питания детектора
//			if( PWR_RES_started == 0 )
//			{
//					POWER_DET_CNTR(PWR_OFF);
//					//sleep_DDS();
//					PWR_RES_started = 1;
//					PWR_RES_FLAG = 1;
//					//HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
//					//DET.IsPowerOn = 0;
//					//HAL_Delay(2000);
//			}
//			else
//				if( TMR4OVF == 1 )
//				{
//					POWER_DET_CNTR(PWR_ON);
			
				if( DET.IsPowerOn == 1 )
				{
						HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
						DET.IsPowerOn = 0;
						SPD.SPD_status_reg &= ~SPD_DET_pwr_flag;
						HAL_Delay(2000);
				}
					HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
					DET.IsPowerOn = 1;
					SPD.SPD_status_reg |= SPD_DET_pwr_flag;
					PWR_RES_started = 0;
					PWR_RES_FLAG = 0;
					HAL_Delay(2000);
					HV_value_set_outp(&HV.HV_target);
					//wakeup_DDS();
					init_DDS();
//				}
		}
		//проверка состояния теплового режима
		if ((tVD_round != T_target)&& (DET.IsPowerOn == 0))
		{
			temp_status = 0;	// не вышел на режим, нет питания на фотодиоде
		}
		if ((tVD_round == T_target)&& (DET.IsPowerOn == 1))
		{
			temp_status = 1;	//вышел на  режим, все нормально
		}
		
		uint8_t Overheat_end_trig;
		if ((tVD >= overheat_achtung.leaving_temp) && (DET.IsPowerOn == 1))
		{
			temp_status = 2;	//слетел с режима, но питание на фотодиоде есть
			Overheat_end_trig = 1;	//флаг перезагрузки МК после сброса температуры радиатора
			HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
			//emerg_cntr = 0;
			SPD.SPD_status_reg &= ~SPD_overheat_flag;
			thrm_zone = 3;					//отключен
			if (overheat_achtung.EN_TIMER_OVERHEAT_FLAG == 1)
			{
			//	MX_IWDG_Init_Overheat();
			}
		}
		
		if ((Overheat_end_trig == 1)&&(ADC_res.CHN2_HEATSINK_THRM <= overheat_achtung.dumping_temp))
		{
			NVIC_SystemReset();
		}
			
						
		frameCounter++;
		//pData->prop_err = PID_st.prp_err;
		//pData->int_err = PID_st.int_err;
		//pData->diff_err = PID_st.dif_err;
		pData->thrm_VD = tVD;
		pData->Bias_VD = HV.HV_Kadc * floats[2];
		float HV_calib = 0.0;
		if( HV_calib_res ((pData->Bias_VD), &HV_calib) )
		{
			pData->Bias_VD = HV_calib ;
			HV.HV_value = HV_calib ;
		}
		////pData->PID_response = response;
		//pData->TEC_current = ADC_res.CHN0_TB3_CURRENT;
		pData->TEC_voltage = ADC_res.CHN1_TB3_VOLTAGE;
		pData->RF_PWR_SNS = ADC_res.CHN3_RF_PWR;
		pData->thrm_heatsink = ADC_res.CHN2_HEATSINK_THRM;
			
		pData->thrm_used = T_target;
		pData->hv_bias_used = HV.HV_target;
		pData->hv_code_used = HV.code ;
		pData->stb_code_used = DDS.STB ;
		pData -> Number_SPD = Detector_number;
		pData -> Ready_SPD = temp_status;
		pData -> Dumping_temp_telemetry = overheat_achtung.dumping_temp;
		pData -> Leaving_temp_telemetry = overheat_achtung.leaving_temp;
		pData -> EN_TIMER_OVERHEAT_telemetry = overheat_achtung.EN_TIMER_OVERHEAT_FLAG;
		pData -> EN_TEC1_telemetry = overheat_achtung.EN_TEC1_FLAG;
		pData -> EN_TEC3_telemetry = overheat_achtung.EN_TEC3_FLAG;
//		pData -> Overheat_temp = SPD.T_emerg;
		
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, USB_TX_Buffer, 64);
		HAL_GPIO_TogglePin(GPIOD, YELLOW_LED_Pin);
		HAL_Delay(15);
		
		
		
		
		
		//проверка перегрева радиатора
		/*if (temp_status == 2)
		{
			Overheat_end_trig = 1;	//флаг перезагрузки МК после сброса температуры радиатора
			HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
			//emerg_cntr = 0;
			SPD.SPD_status_reg &= ~SPD_overheat_flag;
			thrm_zone = 3;					//отключен
		}*/

		

				

					
				
		//USB интерфейс
		if( received )
		{
			lockedReportIndex = newReportIndex;
			reports[lockedReportIndex].id = 1;

			uint8_t cmd = reports[lockedReportIndex].cmd;
			uint8_t *data = reports[lockedReportIndex].data;
			uint8_t reply = 0;              
			switch (cmd)
			{

				
				case CMD_Setup_SPD_controller:
					{
						Setup_SPD_controller_RX_t *rx = (Setup_SPD_controller_RX_t*)data;
			
						if( rx->MCU_RESET != 0 ) 
							HAL_NVIC_SystemReset();
						
						float t_set = rx->Tset;
						float HV_req = (float)(rx->HV_bias)  ;
						uint16_t code_req = (uint16_t)((rx->HV_code) & 0xFFFF);
						uint16_t stb_req = (uint16_t)((rx->DDS_stb_code) & 0x03FF);
						uint16_t dds_readed_code = 0;
/*						float Overheat_temp_read = (float)(rx->Overheat_temp); 
						if (Overheat_temp_read != SPD.T_emerg)
						{
								EEPROM_save_SPD_Overheat_temp = 1;
						}
						SPD.T_emerg = Overheat_temp_read;*/
						
						if(DET.IsPowerOn != 0)
						{
							if( HV_req != (float)0.0 )
							{
								HV_value_set_outp(&HV_req);
								if(HV_req != HV.HV_value)
									EEPROM_save_HV_trg_flag = 1;
							}
							else
							{
								uint16_t code2_req = 0xFFFF - code_req ;
								if(code2_req != HV.code)
								{
									EEPROM_save_HV_trg_flag = 1;
								}
								HV_code_set_outp (&code2_req);
							}
							
							uint8_t resp = 0;
							if(stb_req != DDS.STB)
								EEPROM_save_DDS_STB_flag = 1;
							else
								EEPROM_save_DDS_STB_flag = 0;
							
							DDS_stb_write(stb_req);
							
							for(uint8_t ii = 0 ; (ii < 3)&&(resp == 0); ii++)
							{
								resp = DDS_stb_read(&dds_readed_code);	
								//if(resp == 0)
									//init_DDS();
							}

						}				
						
						if( rx->SPD_reset != 0 ) 
								PWR_RES_FLAG = 1 ;//включаем флаг для последующего сброса питания
						else
								PWR_RES_FLAG = 0 ;
						

							
						if (t_set != T_target)
						{
							if ( (t_set>=(float)(-90.0)) && (t_set<=(float)(0.0)) )
							{
								pwr_set_TB3(T_target);
																
								PID.T_set = t_set;
								T_target = t_set ;
												
								PID_st.int_err = 0;
								PID_st.dif_err = 0;
								PID_st.error_old = 0;
								PID_st.response_mode = 0;	
								
								thrm_capture_trig = 0 ;
								tmr2_en = 1;
								EEPROM_save_T_set_flag = 1;
								
								thrm_zone = 1;
								T_temp_target = T_target ; 
								T_temp_lim = T_lim;
								tmr2_en = 1 ;
								thrm_capture_trig = 0;
								cntr_tim2_rst = 30;
								
								SPD.SPD_status_reg &= SPD_ready_flag ;
								
							}
						}				

						Setup_SPD_controller_TX_t *tx = (Setup_SPD_controller_TX_t*)data;
						tx->Tset = t_set;
						tx->HV_bias = HV.HV_value ;
						tx->HV_code = 0xFFFF - HV.code ;
						tx->DDS_stb_code = dds_readed_code ;
//						tx->Overheat_temp = SPD.T_emerg;
						
											
						reply = 1;
						break;
					}
					
					case CMD_DefaultSetPID:
					{
						
						DefaultSetPID_RX_t *rx_set_PID = (DefaultSetPID_RX_t*)data;
						if(rx_set_PID->SPD_security_key == SPD.secur_key)
						{
							PID.P_coeff = rx_set_PID->p_coeff;
							PID.I_coeff = rx_set_PID->i_coeff;
							PID.D_coeff = rx_set_PID->d_coeff;
							if( (rx_set_PID->PWR_RCVR_DELAY >= 3) && (rx_set_PID->PWR_RCVR_DELAY <= 600 ) )
							{
								cntr_tim2_rst = rx_set_PID->PWR_RCVR_DELAY;
								cntr_tim2_val = cntr_tim2_rst;
							}
							//сохранение в EEPROM
						
							addr_dest = BASE_DEV_DESC + BASE_PID_SET;
							ptr_src = &PID;
							size_src = sizeof(PID);
							sta = 0;
							sta = copy_to_eeprom(addr_dest, ptr_src, size_src);//записываем значения
						}
												
						DefaultSetPID_TX_t *tx_calibr = (DefaultSetPID_TX_t*)data;
						if (sta==1)
						{
							tx_calibr->p_coeff = PID.P_coeff;
							tx_calibr->i_coeff = PID.I_coeff;
							tx_calibr->d_coeff = PID.D_coeff;
							tx_calibr->T_set = PID.T_set;
							tx_calibr->T_err = PID.T_err;
							tx_calibr->PWR_RCVR_DELAY = cntr_tim2_rst;
							tx_calibr->adc_filtr = (uint32_t)HV.code;
							HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
						}
						else
						{
							tx_calibr->p_coeff = 0;
							tx_calibr->i_coeff = 0;
							tx_calibr->d_coeff = 0;
							tx_calibr->T_set = 0;
							tx_calibr->T_err = 0;
						}
						reply = 1;
						break;
					}	
					
					
					case CMD_CalibThermometr:
					{
						CalibThermometr_RX_t *rx_CalibThermometr = (CalibThermometr_RX_t*)data;
						if(rx_CalibThermometr->SPD_security_key == SPD.secur_key)
						{
							//rx_CalibThermometr->tg_tmpr_corr
							//rx_CalibThermometr->ofs_tmpr_corr;
							//rx_CalibThermometr->Kr_ret;
							//rx_CalibThermometr->R0_ofs;
						}
						
	
						
						reply = 1;
						break;
					}	
					
					
					case CMD_SetDCDC:
					{
						SetDCDC_RX_t *rx_dcdc = (SetDCDC_RX_t*)data;
//						if(rx_dcdc->SPD_security_key == SPD.secur_key)
//						{
//							PID.dc_voltage_eps = rx_dcdc->dc_voltage_eps;
//							PID.max_dc_volt = rx_dcdc->max_dc_volt;
//							PID.max_dc_curr = rx_dcdc->max_dc_curr;
//						}
						
						SetDCDC_TX_t *tx_dcdc = (SetDCDC_TX_t*)data;
						tx_dcdc->dc_voltage_eps = PID.dc_voltage_eps;
						tx_dcdc->max_dc_volt = PID.max_dc_volt;
						tx_dcdc->max_dc_curr = PID.max_dc_curr;

						reply = 1;
						break;
					}	
					
					
					case CMD_SetHV:
					{

						SetHV_RX_t *rx_hv = (SetHV_RX_t*)data;
//						if(rx_hv->SPD_security_key == SPD.secur_key)
//						{
//							HV.HV_target = rx_hv->HV_bias_preset;
//							HV.HV_max = rx_hv->HV_bias_damage_VD;
//							HV.tg_calib = rx_hv->HV_slope;
//							HV.ofs_calib = rx_hv->HV_offset;
//						}
						
						SetHV_TX_t *tx_hv = (SetHV_TX_t*)data;
						tx_hv->HV_bias_max = HV.HV_max;
						tx_hv->HV_bias_preset = HV.HV_target;
						tx_hv->HV_slope = HV.HV_Kadc;
						
						
						
						   
						reply = 1;
						break;
					}	
					
							
					case CMD_SPD_DDS:
					{
//						SPD_DDS_RX_t *rx_dds = (SPD_DDS_RX_t*)data;
//						if(rx_dds->SPD_security_key == SPD.secur_key)
//						{
//							DDS.FTW = (rx_dds->DDS_FTW)&(0xFFFFFFFFFFFF);
//							DDS.PHS = (rx_dds->DDS_PHS)&(0x3FFF);
//						}
//						///запись данных в регистры DDS
//						if(DET.IsPowerOn == 1)
//						{
						//							для отладки функции нужна методика проверки
//							
//						}
						
						SPD_DDS_TX_t *tx_dds = (SPD_DDS_TX_t*)data;
						tx_dds->DDS_FTW = DDS.FTW;
						tx_dds->DDS_PHS = DDS.PHS;
					
						reply = 1;
						break;
					}	
					
				case CMD_SerialNumber:
				{
					CMD_SerialNumber_RX_t *rx_set = (CMD_SerialNumber_RX_t*)data;
						if(rx_set->SPD_security_key == SPD.secur_key)
						{
							
							SPD.serial_number = rx_set->serial_number_SPD;
							SPD.soft_version = rx_set->SPD_soft_version;
							SPD.date_day = rx_set->date_day;
							SPD.date_month = rx_set->date_month;
							SPD.date_year = rx_set->date_year;
							
							
							CMD_SerialNumber_TX_t *tx_set = (CMD_SerialNumber_TX_t*)data;
							//tx_set = (CMD_SerialNumber_TX_t*)rx_set ;
							
							tx_set->serial_number_SPD = SPD.serial_number ;
							tx_set->SPD_soft_version = SPD.soft_version ;
							tx_set->date_day = SPD.date_day;
							tx_set->date_month = SPD.date_month ;
							tx_set->date_year = SPD.date_year ;
							
													
							EEPROM_save_SPD_serial_flag = 1;
						}
						else
						{
							CMD_SerialNumber_TX_t *tx_set = (CMD_SerialNumber_TX_t*)data;
							//чтение данных из EEPROM
							SPD_regs SPD_rd ;
							addr_src = BASE_SPD_SERIAL ;
							size_src = sizeof(SPD_rd);
							sta = copy_from_eeprom(addr_src, &SPD_rd, size_src);
							if (sta == 1)
							{
								SPD.SPD_status_reg &= (~SPD_EEPROM_err_flag);
															
								tx_set->serial_number_SPD = SPD_rd.serial_number;
								tx_set->SPD_soft_version = SPD_rd.soft_version;
								tx_set->date_day = SPD_rd.date_day;
								tx_set->date_month = SPD_rd.date_month;
								tx_set->date_year = SPD_rd.date_year;
							}
							else
							{
								SPD.SPD_status_reg |= SPD_EEPROM_err_flag;
								
								tx_set->serial_number_SPD = 0;
								tx_set->SPD_soft_version = 0;
								tx_set->date_day = 0;
								tx_set->date_month = 0;
								tx_set->date_year = 0;
							}
						}
						
						
						
						reply = 1;
						break;
				}
				
				case CMD_SPD_NAME_WRITE:
				{
					SPD_name_write_RX_t *rx = (SPD_name_write_RX_t*)data;
					
						uint8_t *ptr = data;
						for(uint8_t ii = 0; ii < 63 ; ii++)
							str_SPD_name[ii] = *(ptr + ii);
						
						EEPROM_save_SPD_name_flag = 1;
					
				
					SPD_name_write_TX_t *tx = (SPD_name_write_TX_t*)data;
					
					tx = (SPD_name_write_TX_t*)rx;
					reply = 1 ;
					break;
				}
				
				case CMD_SPD_NAME_READ:
				{
					uint8_t *ptr_byte = reports[lockedReportIndex].data ;
					for(uint8_t ii = 0; ii < 63; ii++)
					{
						addr_dest = STR_SPD_DESC + ii;
						str_SPD_name[ii] = 0 ;
						eeprom_read(addr_dest, &str_SPD_name[ii]);
						*(ptr_byte + ii) = str_SPD_name[ii] ;
						
					}
					
					reply = 1 ;
					break;
				}
				
				case CMD_SPD_Status:
				{
					//проверка флагов для заполнения статуса
					if( SPD.SPD_status_reg &  0xFFFFFFF0) 
						SPD.SPD_status_reg |= SPD_err_flag ;
					else
						SPD.SPD_status_reg &= ~SPD_err_flag ;
		
					SPD_status_TX_t *tx = (SPD_status_TX_t*)data;
					tx->SPD_status = SPD.SPD_status_reg ;
					
					reply = 1;
					break;
				}
				case CMD_Number_SPD:
				{
					// from host 
					Number_SPD_RX_t *rx = (Number_SPD_RX_t*)data;
					Detector_number = rx -> SPD_num;			
					EEPROM_save_SPD_number = 1;
					
					reply = 1;
					break;
				}
				case CMD_Ahtung:
				{
					// from host 
					
					float Dumping_temp_read;
					float Leaving_temp_read;
					uint8_t EN_TIMER_OVERHEAT_read;
					uint8_t EN_TEC1_FLAG_read;
					uint8_t EN_TEC3_FLAG_read;
					
					Ahtung_RX_t *rx = (Ahtung_RX_t*)data;
					Dumping_temp_read = (float) (rx -> Dumping_temp);
					Leaving_temp_read = (float)(rx -> Leaving_temp);		
//					EN_TIMER_OVERHEAT_read = rx -> EN_TIMER_OVERHEAT;
//					EN_TEC1_FLAG_read = rx -> EN_TEC1_FLAG;
//					EN_TEC3_FLAG_read = rx -> EN_TEC3_FLAG;
					

					
					
					//проверка на запись в EEPROM
					
					if (overheat_achtung.dumping_temp != Dumping_temp_read)
					{
						EEPROM_save_dumping_temp_flag = 1;
					}
					if (overheat_achtung.leaving_temp != Leaving_temp_read)
					{
						EEPROM_save_leaving_temp_flag = 1;
					}
					if (overheat_achtung.EN_TIMER_OVERHEAT_FLAG != EN_TIMER_OVERHEAT_read)
					{
						EEPROM_save_EN_TIMER_OVERHEAT_FLAG_flag = 1;
					}
					
					overheat_achtung.dumping_temp = Dumping_temp_read;
					overheat_achtung.leaving_temp = Leaving_temp_read;
					overheat_achtung.EN_TIMER_OVERHEAT_FLAG = EN_TIMER_OVERHEAT_read;
					overheat_achtung.EN_TEC1_FLAG = EN_TEC1_FLAG_read;
					overheat_achtung.EN_TEC3_FLAG	= EN_TEC3_FLAG_read;
					
					//to host
					Ahtung_TX_t *tx = (Ahtung_TX_t*)data;
					tx->Dumping_temp = overheat_achtung.dumping_temp;
					tx->Leaving_temp = Leaving_temp_read;
//					tx->EN_TIMER_OVERHEAT = EN_TIMER_OVERHEAT_read;
//					tx->EN_TEC1_FLAG = overheat_achtung.EN_TEC1_FLAG;
//					tx->EN_TEC3_FLAG = overheat_achtung.EN_TEC3_FLAG;
					
					reply = 1;
					break;
					
				}
		
				break;
										
					default:
						break;
				
				
				}
				
				received = 0;

				if (reply)
				{
					reply = 0;
					USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)(reports + lockedReportIndex), 64);
					HAL_Delay(15);
				}
				//HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
				
				if(EEPROM_save_T_set_flag==1)
				{
					EEPROM_save_T_set_flag = 0;
					//сохранение в EEPROM
					addr_dest = BASE_T_SET;
					uint8_t buffer_wr[4];
					//int res_wr = (int)(T_target * (float)1000.0 * (float)(-1.0));
					float *ptr_float = &T_target;
					uint32_t res_wr = *(uint32_t*)ptr_float;
					uint32_t temp_res = res_wr & 0xFF ;
					buffer_wr[0] = (uint8_t)temp_res;
					temp_res = (res_wr & 0xFF00) >> 8 ;
					buffer_wr[1] = (uint8_t)temp_res;
					temp_res = (res_wr & 0xFF0000) >> 16 ;
					buffer_wr[2] = (uint8_t)temp_res;
					temp_res = (res_wr & 0xFF000000) >> 24 ;
					buffer_wr[3] = (uint8_t)temp_res;
					uint8_t *ptr_src = buffer_wr;
					for (uint16_t ii = 0; ii < sizeof(buffer_wr); ii++)
					{
						addr_dest = BASE_T_SET + ii;
						eeprom_write(addr_dest, ptr_src);
						ptr_src++;
						HAL_Delay(12);
					}
				}
				if(EEPROM_save_HV_trg_flag == 1  )
				{
					EEPROM_save_HV_trg_flag = 0;
					//сохранение в EEPROM
					uint8_t buffer_wr[2];
					uint8_t *ptr_src = buffer_wr;
					int res_wr = (int)(HV.code);
					uint32_t temp_res = res_wr & 0xFF ;
					buffer_wr[0] = (uint8_t)temp_res;
					temp_res = (res_wr & 0xFF00) >> 8 ;
					buffer_wr[1] = (uint8_t)temp_res;
					
					for (uint16_t ii = 0; ii < 2; ii++)
					{
						addr_dest = BASE_HV_SET + ii;
						eeprom_write(addr_dest, ptr_src);
						ptr_src++;
						HAL_Delay(12);
					}	
				}
				if(EEPROM_save_DDS_STB_flag == 1)
				{
					EEPROM_save_DDS_STB_flag = 0;
					
					//сохранение в EEPROM
					uint8_t buffer_wr[2];
					uint8_t *ptr_src = buffer_wr;
					int res_wr = (int)(DDS.STB);
					uint32_t temp_res = res_wr & 0xFF ;
					buffer_wr[0] = (uint8_t)temp_res;
					temp_res = (res_wr & 0xFF00) >> 8 ;
					buffer_wr[1] = (uint8_t)temp_res;
					
					for (uint16_t ii = 0; ii < 2; ii++)
					{
						addr_dest = BASE_STB_SET + ii;
						eeprom_write(addr_dest, ptr_src);
						ptr_src++;
						HAL_Delay(12);
					}	
					
				}
				
				
				if(EEPROM_save_SPD_name_flag == 1)
				{
					EEPROM_save_SPD_name_flag = 0;
					uint8_t byte_readed = 0;
					for(uint8_t ii = 0; ii < 63; ii++)
					{
						addr_dest = STR_SPD_DESC + ii;
						for(uint8_t jj = 0 ; jj < 3; jj++)
						{
							eeprom_write(addr_dest, &str_SPD_name[ii]);
							HAL_Delay(10);
							byte_readed = 0;
							eeprom_read(addr_dest, &byte_readed);
							if(byte_readed == str_SPD_name[ii])
								break;
						}
					}
					
				}
				
				if (EEPROM_save_SPD_serial_flag == 1)
				{
					EEPROM_save_SPD_serial_flag = 0;
					SPD_regs SPD_wr;
					
					SPD_wr = SPD ;
					SPD_wr.state = 0;
					
					addr_src = BASE_SPD_SERIAL ;
					size_src = sizeof(SPD_wr);
					sta = copy_to_eeprom(addr_src, &SPD_wr, size_src);
					if (sta == 1)
						SPD.SPD_status_reg &= (~SPD_EEPROM_err_flag);
					else
						SPD.SPD_status_reg |= SPD_EEPROM_err_flag;
				}
				
				if (EEPROM_save_SPD_number == 1)
				{
					EEPROM_save_SPD_number = 0;
					Detector_number_wr = Detector_number;
					addr_src = BASE_SPD_NUMBER ;
					size_src = sizeof(Detector_number_wr);
					sta = copy_to_eeprom(addr_src, &Detector_number_wr, size_src);
					if (sta == 1)
						SPD.SPD_status_reg &= (~SPD_EEPROM_err_flag);
					else
						SPD.SPD_status_reg |= SPD_EEPROM_err_flag;		
				}
					
				if (EEPROM_save_dumping_temp_flag == 1)
				{
					EEPROM_save_dumping_temp_flag = 0;
					addr_src = BASE_DUMPING_TEMP;
					size_src = sizeof(overheat_achtung.dumping_temp);
					sta = copy_to_eeprom(addr_src, &overheat_achtung.dumping_temp, size_src);
					if (sta == 1)
						SPD.SPD_status_reg &= (~SPD_EEPROM_err_flag);
					else
						SPD.SPD_status_reg |= SPD_EEPROM_err_flag;					
				}
				
	/*			if (EEPROM_save_EN_TIMER_OVERHEAT_FLAG_flag == 1)
				{
					EEPROM_save_EN_TIMER_OVERHEAT_FLAG_flag = 0;
					addr_src = BASE_EN_TIMER_OVERHEAT_FLAG;
					size_src = sizeof(overheat_achtung.EN_TIMER_OVERHEAT_FLAG);
					sta = copy_to_eeprom(addr_src, &overheat_achtung.EN_TIMER_OVERHEAT_FLAG, size_src);
					if (sta == 1)
						SPD.SPD_status_reg &= (~SPD_EEPROM_err_flag);
					else
						SPD.SPD_status_reg |= SPD_EEPROM_err_flag;					
				}*/
				
				if (EEPROM_save_leaving_temp_flag == 1)
				{
					EEPROM_save_leaving_temp_flag = 0;
					addr_src = BASE_LEAVING_TEMP;
					size_src = sizeof(overheat_achtung.leaving_temp);
					sta = copy_to_eeprom(addr_src, &overheat_achtung.leaving_temp, size_src);
					if (sta == 1)
						SPD.SPD_status_reg &= (~SPD_EEPROM_err_flag);
					else
						SPD.SPD_status_reg |= SPD_EEPROM_err_flag;					
				}
			}//if received
////			time_stop = HAL_GetTick();
////			time_delta = time_stop - time_start;
			
	}
	
	/* USER CODE END WHILE */
  
	

	/* USER CODE BEGIN 3 
	while(1)
	{
	
	}
   USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
		//HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
		HAL_NVIC_SetPriority(SysTick_IRQn, 1, 1);
		HAL_NVIC_SetPriority(OTG_FS_IRQn, 1, 2);
		HAL_NVIC_SetPriority(TIM2_IRQn,1,3);	
		//HAL_NVIC_SetPriority(TIM3_IRQn,2,1);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 30000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 30000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;//1s
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


/* CRC init function */
void MX_CRC_Init(void)
{		
		hcrc.Instance = CRC;
		if (HAL_CRC_Init(&hcrc) != HAL_OK) {
				Error_Handler();
		}
		
		__HAL_RCC_CRC_CLK_ENABLE();
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 32;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	


}

uint8_t select_chn_ADC(uint8_t channel)
{
	channel &= 0x3;
	switch (channel)
	{
		case 0:
		{
			adc_config.Channel = ADC_CHANNEL_0;
			adc_config.Rank = 1;
			adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK)
			{
				return 0;
			}
			break;
		}
		
		case 1:
		{
			adc_config.Channel = ADC_CHANNEL_1;
			adc_config.Rank = 1;
			adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK)
			{
				return 0;
			}
			
			break;
		}		
		
		case 2:
		{
			adc_config.Channel = ADC_CHANNEL_2;
			adc_config.Rank = 1;
			adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK)
			{
				return 0;
			}
			
			break;
		}
		
		case 3:
		{
			adc_config.Channel = ADC_CHANNEL_3;
			adc_config.Rank = 1;
			adc_config.SamplingTime = ADC_SAMPLETIME_28CYCLES;
			if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK)
			{
				return 0;
			}
			
			break;
		}
	
	}

	return 1;
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

//  hspi4.Instance = SPI4;
//  hspi4.Init.Mode = SPI_MODE_MASTER;
//  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi4.Init.NSS = SPI_NSS_SOFT;
//  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
//  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi4.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi4) != HAL_OK)
//  {
//    Error_Handler();
//  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPI4_SCK_M_Pin|SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(GPIOE, SPI4_CS1_Pin|SPI4_CS2_Pin|SPI4_RST_Pin|SPI4_CS0_Pin, GPIO_PIN_SET);
	
	
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI4_PWDN_Pin|I2C3_RST_Pin|EEPROM_WC_Pin|TP_SPI4_Pin 
                          |TP_SPI1_Pin|TP_I2C3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, SPI4_GT_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin|RED_LED_Pin|GREEN_LED_Pin|TP_I2C1_Pin| 
                          PWR_DET_CNTR_Pin|SPI1_CS_ADC_Pin|SPI1_SYNC_ADC_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(GPIOD, PWR_TEC_CNTR_Pin|PWR_BASE_CNTR_Pin, GPIO_PIN_SET);												
													
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, I2C1_RST_Pin, GPIO_PIN_RESET);

////инициализация обычных выходов
//  /*Configure GPIO pins : SPI4_CS1_Pin SPI4_CS2_Pin SPI4_RST_Pin SPI4_CS0_Pin */
//  GPIO_InitStruct.Pin = SPI4_SCK_M_Pin|SPI4_MOSI_M_Pin|SPI4_CS0_Pin|SPI4_CS1_Pin|SPI4_CS2_Pin|SPI4_RST_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	//инициализация выходов с открытым коллектором для работы с EXT_DAC of HV
  /*Configure GPIO pins : SPI4_CS1_Pin SPI4_CS2_Pin SPI4_RST_Pin SPI4_CS0_Pin */
  GPIO_InitStruct.Pin = SPI4_SCK_M_Pin|SPI4_MOSI_M_Pin|SPI4_CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	//обычные выходы
	GPIO_InitStruct.Pin = SPI4_CS1_Pin|SPI4_CS2_Pin|SPI4_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = SPI4_MISO_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI4_PWDN_Pin I2C3_RST_Pin EEPROM_WC_Pin TP_SPI4_Pin 
                           TP_SPI1_Pin TP_I2C3_Pin */
  GPIO_InitStruct.Pin = SPI4_PWDN_Pin|SPI4_GT_ON_Pin|I2C3_RST_Pin|EEPROM_WC_Pin|TP_SPI4_Pin|TP_SPI1_Pin|TP_I2C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHRED0_Pin SHRED1_Pin SHRED2_Pin SHRED3_Pin */
  GPIO_InitStruct.Pin = SHRED0_Pin|SHRED1_Pin|SHRED2_Pin|SHRED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : YELLOW_LED_Pin RED_LED_Pin GREEN_LED_Pin TP_I2C1_Pin 
                           PWR_TEC_CNTR_Pin PWR_BASE_CNTR_Pin PWR_DET_CNTR_Pin SPI1_CS_ADC_Pin 
                           SPI1_SYNC_ADC_Pin */
  GPIO_InitStruct.Pin = YELLOW_LED_Pin|RED_LED_Pin|GREEN_LED_Pin|TP_I2C1_Pin 
                          |PWR_TEC_CNTR_Pin|PWR_BASE_CNTR_Pin|PWR_DET_CNTR_Pin|SPI1_CS_ADC_Pin 
                          |SPI1_SYNC_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_RDY_LVL_Pin */
  GPIO_InitStruct.Pin = SPI1_RDY_LVL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_RDY_LVL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG_TEC_Pin */
  GPIO_InitStruct.Pin = PG_TEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PG_TEC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_RDY_ADC_Pin */
  GPIO_InitStruct.Pin = SPI1_RDY_ADC_Pin;
  //GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_RDY_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_RST_Pin SPI4_GT_ON_Pin */
  GPIO_InitStruct.Pin = I2C1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 0x0FFF;
  /*if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }*/

}

uint8_t POWER_DET_CNTR(uint8_t power_state)
{
//	if ( power_state == (uint8_t)DET.IsPowerOn )
//		return 0;
//	else
//	{
		if (power_state == 0)
		{
			//HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
			query_det_pwr = 0;//query to turn OFF
		}
		else
		{
			//HAL_GPIO_WritePin(GPIOD, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
			query_det_pwr = 1;//query to turn ON
		}
//	}
	
	if(tmr4_started == 1)
	{
		if (TMR4OVF == 1)
		{
			cntr_tim4_val = cntr_tim4_rst;
			tmr4_started = 0;
			HAL_NVIC_DisableIRQ(TIM4_IRQn);
			HAL_TIM_Base_Stop_IT(&htim4);
			if(query_det_pwr == 0)
				HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_RESET);//detector power OFF
			else
				HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
		}
	}
	else
	{
		TMR4OVF = 0;
		cntr_tim4_val = cntr_tim4_rst;
		tmr4_started = 1;
		//запуск таймера защитного интервала
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		HAL_TIM_Base_Start_IT(&htim4);
		
	}
	return 1;//вкл/выкл по истечении защитного интервала
}

void POWER_TEC_TB1_CNTR(uint8_t power_state)
//	void POWER_TEC_TB1_CNTR(uint8_t power_state, uint8_t power_value)
{
	if (power_state==0)
	{
			HAL_GPIO_WritePin(GPIOD, PWR_BASE_CNTR_Pin, GPIO_PIN_SET);//TEC TB1 OFF
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, PWR_BASE_CNTR_Pin, GPIO_PIN_RESET);//TEC TB1 ON
	}
}


void POWER_TEC_TB3_CNTR(uint8_t power_state)
{
	if (power_state==0)
			HAL_GPIO_WritePin(GPIOD, PWR_TEC_CNTR_Pin, GPIO_PIN_SET);//TEC TB3 OFF
	else
			HAL_GPIO_WritePin(GPIOD, PWR_TEC_CNTR_Pin, GPIO_PIN_RESET);//TEC TB3 ON
}

float convert(uint8_t index, uint32_t value)
{
		float result = 0.0;
		float factor = 1.0;
		float offset = 0.0;
		
		if (index==0)
		{
				factor = ch_thrm.factor;
				offset = ch_thrm.offset;
		}
		
		if (index==1)
		{
				factor = ch_curr.factor;
				offset = ch_curr.offset;
		}
				
		if (index==2)
		{
				factor = ch_hv.factor;
				offset = ch_hv.offset;
		}
					
		result = (float)(value & 0x0FFFFFF) * factor;
					
		return (result + offset);
}

uint32_t takeData(uint8_t index, float* dst)
{
		uint32_t code = 0;
			if (Get(&code, &_fifos[index])) 
			{
				*dst = convert(index, code);
				return 1;
			}
		return 0;
}

float takeTherm(float value_pt1000)
{
	//реализация аппроксимации полиномом 3й степени
	float arg = value_pt1000;
	float summ = vect_coeff[0];
	for (uint32_t i = 1 ; i < 4 ; i++)
	{
		summ += arg * vect_coeff[i];
		arg *= value_pt1000;
	}
	
	return summ;
}



inline float calc_thrm_heatsink(float voltage)
{
	return ( (float)(-1481.96) +  (float)1000.0 * sqrt(( (float)2.1962 + ((float)1.8639 - voltage)/(float)3.88 )  ) );
}


float abs_value(uint8_t *sign, float value)
{
	if(value >= (float)0.0)
	{
		*sign = 0;
		return value;
	}
	else
	{
		*sign = 1;	
		return (value * (float)(-1.0));
	}
}

uint8_t control_version(void)
{
	uint8_t target_version = (uint8_t)SOFTWARE_VERSION;
	uint8_t read_version = 0;
	
	if(HAL_GPIO_ReadPin(GPIOB, SHRED0_Pin)==GPIO_PIN_SET) 
			read_version |= 0x1;
	if(HAL_GPIO_ReadPin(GPIOB, SHRED1_Pin)==GPIO_PIN_SET) 
			read_version |= 0x2;
	if(HAL_GPIO_ReadPin(GPIOB, SHRED2_Pin)==GPIO_PIN_SET) 
			read_version |= 0x4;
	if(HAL_GPIO_ReadPin(GPIOB, SHRED3_Pin)==GPIO_PIN_SET) 
			read_version |= 0x8;
	
	if (read_version == target_version)
		return 1;
	else
		return 0;
}	

uint8_t pwr_set_TB3(float thrm)
{
	if(thrm > (float)(-5.0)) tec_pwr_lim  = 10 ;
	else
		if((thrm <= (float)(-5.0)) && (thrm >(float)(-15.0) ) ) tec_pwr_lim = 50 ;
		else
			if((thrm <= (float)(-15.0)) && (thrm >(float)(-30.0) ) ) tec_pwr_lim = 60 ;
			else
				if((thrm <= (float)(-30.0)) && (thrm >(float)(-40.0) ) ) tec_pwr_lim = 80 ;
				else
					if((thrm <= (float)(-40.0)) && (thrm >(float)(-50.0) ) ) tec_pwr_lim = 100 ;
					else
						if((thrm <= (float)(-50.0)) && (thrm >(float)(-90.0) ) ) tec_pwr_lim = 100 ;
						else
							return 0 ;

	return 1;
}

//функция проверки нахождения в пределах интервала температур
uint8_t thrm_inrange(float thrm_inp, float thrm_trg, float thrm_lim)
{
	uint8_t sign = 0;
	if ( abs_value(&sign, (thrm_inp - thrm_trg)) <= thrm_lim )
		return 1;
	else
		return 0;
}

//функция проверки стационарности температуры в пределах временных и температурных границ
uint8_t thrm_capture(float thrm_inp, float thrm_trg, float thrm_lim)
{
	uint8_t thrm_capt_flag = 0;
	uint8_t sign = 0;
	
	if ( abs_value(&sign, (thrm_inp - thrm_trg)) <= thrm_lim )
	{
		if(tmr2_en == 1)//если запуск разрешен..
		{
			if( tim2_started == 0 )//.. и таймер не запущен
			{
				thrm_capt_flag = 0;
				tim2_started = 1;
				HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET) ;
				//иниц/я и запуск таймера
				TMR2OVF = 0 ;
				cntr_tim2_val = cntr_tim2_rst;
				HAL_NVIC_EnableIRQ(TIM2_IRQn);
				HAL_TIM_Base_Start_IT(&htim2);
			}
			else//таймер уже запущен
			{
				if( TMR2OVF == 1 )
				{
					thrm_capt_flag = 1;
					TMR2OVF = 0;
					tmr2_en = 0;//запрещение таймера
					tim2_started = 0;//запоминаем что таймер был остановлен
					HAL_NVIC_DisableIRQ(TIM2_IRQn);
					HAL_TIM_Base_Stop_IT(&htim2);
					HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET) ;			
				}
			}
		
		}
	}		
	else//температура вышла за пределы границ
	{
		thrm_capt_flag = 0 ;
		tmr2_en = 1 ;
		tim2_started = 0 ;
		TMR2OVF = 0 ;
		cntr_tim2_val = cntr_tim2_rst;
		HAL_NVIC_DisableIRQ(TIM2_IRQn);
		HAL_TIM_Base_Stop_IT(&htim2);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET) ;
	}				

	return thrm_capt_flag ;
}

inline float calc_rf_pwr(float inp)
{
	return ( inp * vect_rfpwr_coeff[0] + vect_rfpwr_coeff[1] );
}


uint8_t SDADC_conv (uint8_t chn, uint32_t *readed_data )
{
	//uint8_t result = 0;
	unsigned char temp_status = 0;
	uint8_t curr_chn;
	uint8_t ready = 0;
	uint8_t fault_reading = 0;
	//uint8_t ignore_chn_selector = 0;
	//uint32_t *ptr_readed_data = readed_data;
	
	GPIO_PinState RDY_LEVEL_STATE = GPIO_PIN_SET;
			
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_GPIO_WritePin(GPIOC, TP_SPI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);

	do 
	{
		curr_chn = chn;
		////RDY_LEVEL_STATE = HAL_GPIO_ReadPin(SPI1_RDY_LVL_ADC_GPIO_Port, SPI1_RDY_LVL_ADC_Pin);
		RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
		if(RDY_LEVEL_STATE == GPIO_PIN_RESET)
		{

			*readed_data = ADC_read_DATA( &curr_chn, &temp_status);

			
				if (curr_chn == chn)
				{
					if ( takeData(chn, &floats[chn])  )
					{
							ready = 1;
							break;
					}
				}
				else
				{
					takeData(curr_chn, &floats[curr_chn]);
					ready = 0;
					fault_reading++;
					if (fault_reading > 10)//РµСЃР»Рё С‡РёСЃР»Рѕ СЃС‡РёС‚С‹РІР°РЅРёР№ РЅРµСЃРѕРѕС‚РІРµС‚СЃРІСѓСЋС‰РµРіРѕ РєР°РЅР°Р»Р° РІС‹С€Рµ 10, С‚Рѕ РІС‹С…РѕРґ
						break;
				}
			
		

		}
	} while (1);
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(GPIOC, TP_SPI1_Pin, GPIO_PIN_RESET);

	
	return ready;		
}


void tim3_delay(void)
{
	TMR3OVF = 0 ;
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
	HAL_TIM_Base_Start_IT(&htim3);
	while(TMR3OVF == 0 ) ;
	//HAL_GPIO_TogglePin(TP_SPI4_GPIO_Port, TP_SPI4_Pin);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	//выключение TEC элементов
	POWER_TEC_TB1_CNTR(PWR_OFF);
	POWER_TEC_TB3_CNTR(PWR_OFF);
	//выключение питания детектора
	POWER_DET_CNTR(PWR_OFF);

	HAL_GPIO_WritePin(GPIOD, YELLOW_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(300);
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(300);
		//HAL_NVIC_SystemReset();
		//HAL_IWDG_Refresh(&hiwdg);	
		
		//USB отправка статуса ДОФ и при необходимости перзугрузка при запросе
		//.....
  }
  /* USER CODE END Error_Handler */ 
}
/*static void MX_IWDG_Init_Overheat(void)		//таймер перезагрузки МК после сброса температуры с радиатора
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 625;		//
	HAL_IWDG_Init(&hiwdg);
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}*/


#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
