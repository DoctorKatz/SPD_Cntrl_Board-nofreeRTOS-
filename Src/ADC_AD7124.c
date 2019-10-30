#include "stm32f4xx_hal.h"
#include "ADC_AD7124.h"
#include "fifo_buffer.h"
#include "SPD_controller.h"


#define ADC_GAIN_CHN0 0x4//коэффициент усиления напряжения термодатчика 16 код 4
#define ADC_GAIN_CHN1 0x4//коэффициент усиления тока термодатчика 16 код 4
#define ADC_ID_OK 0x02
#define timeout_spi 0x10000
#define SPI_CRC_CALC 0//use CRC calc
#define SPI_ERR_CNTR 10//cntr of SPI fault
#define waiting_cycles 30

//декларирование регистров типа структура
REG_STRUCT 	AD7124_Status;
REG_STRUCT 	AD7124_ADC_Control;
REG_STRUCT 	AD7124_Data;
REG_STRUCT 	AD7124_IOCon1;
REG_STRUCT 	AD7124_IOCon2;
REG_STRUCT 	AD7124_ID;
REG_STRUCT 	AD7124_Error;
REG_STRUCT 	AD7124_Error_En;
REG_STRUCT 	AD7124_Mclk_Count;
REG_STRUCT 	AD7124_Channel_0;
REG_STRUCT 	AD7124_Channel_1;
REG_STRUCT 	AD7124_Channel_2;
REG_STRUCT 	AD7124_Channel_3;
REG_STRUCT 	AD7124_Channel_4;
REG_STRUCT 	AD7124_Channel_5;
REG_STRUCT 	AD7124_Channel_6;
REG_STRUCT 	AD7124_Channel_7;
REG_STRUCT 	AD7124_Channel_8;
REG_STRUCT 	AD7124_Channel_9;
REG_STRUCT 	AD7124_Channel_10;
REG_STRUCT 	AD7124_Channel_11;
REG_STRUCT 	AD7124_Channel_12;
REG_STRUCT 	AD7124_Channel_13;
REG_STRUCT 	AD7124_Channel_14;
REG_STRUCT 	AD7124_Channel_15;	
REG_STRUCT 	AD7124_Config_0;
REG_STRUCT 	AD7124_Config_1;
REG_STRUCT 	AD7124_Config_2;
REG_STRUCT 	AD7124_Config_3;
REG_STRUCT 	AD7124_Config_4;
REG_STRUCT 	AD7124_Config_5;
REG_STRUCT 	AD7124_Config_6;
REG_STRUCT 	AD7124_Config_7;
REG_STRUCT 	AD7124_Filter_0;
REG_STRUCT 	AD7124_Filter_1;
REG_STRUCT 	AD7124_Filter_2;
REG_STRUCT 	AD7124_Filter_3;
REG_STRUCT 	AD7124_Filter_4;
REG_STRUCT 	AD7124_Filter_5;
REG_STRUCT 	AD7124_Filter_6;
REG_STRUCT 	AD7124_Filter_7;
REG_STRUCT 	AD7124_Offset_0;
REG_STRUCT 	AD7124_Offset_1;
REG_STRUCT 	AD7124_Offset_2;
REG_STRUCT 	AD7124_Offset_3;
REG_STRUCT 	AD7124_Offset_4;
REG_STRUCT 	AD7124_Offset_5;
REG_STRUCT 	AD7124_Offset_6;
REG_STRUCT 	AD7124_Offset_7;
REG_STRUCT 	AD7124_Gain_0;
REG_STRUCT 	AD7124_Gain_1;
REG_STRUCT 	AD7124_Gain_2;
REG_STRUCT 	AD7124_Gain_3;
REG_STRUCT 	AD7124_Gain_4;
REG_STRUCT 	AD7124_Gain_5;
REG_STRUCT 	AD7124_Gain_6;
REG_STRUCT 	AD7124_Gain_7;

REG_STRUCT *ptr_struct;

extern SPI_HandleTypeDef hspi1;
extern void Error_Handler(void);
extern uint8_t tx8;
extern uint8_t tx16[2];
extern uint8_t tx24[3];
extern uint8_t tx32[4];

extern uint8_t rx8;
extern uint8_t rx16[2];
extern uint8_t rx24[3];
extern uint8_t rx32[4];

//extern float floats[3];
extern FIFO_BUFF _fifos[FIFO_COUNT];
extern HV_src HV;

extern uint32_t _result;
extern void EXTI9_5_IRQHandler(void);

//коэффициенты преобразования каналов
line_coeff ch_thrm;
line_coeff ch_curr;
line_coeff ch_hv;

uint32_t AD7124_OFFSET0 = 0x800000;
uint32_t AD7124_OFFSET1 = 0x800000;
uint32_t AD7124_OFFSET2 = 0x800000;
uint32_t AD7124_GAIN0 = 0x500000;
uint32_t AD7124_GAIN1 = 0x500000;
uint32_t AD7124_GAIN2 = 0x500000;

uint32_t AD7124_CALIB_SET[6];

//инициализация регистров для работы с АЦП
void ADC_RG_INIT (void);
uint8_t ADC_SOFT_RESET(void);
void ADC_HARD_RESET(void);
uint8_t SPI_response(HAL_StatusTypeDef SPI_TRX_Status);
//коммуникация по SPI
uint8_t ADC_write(REG_STRUCT *ptr_struct, uint32_t data);
uint8_t ADC_read (REG_STRUCT *ptr_struct, uint32_t *data);
uint32_t ADC_read_DATA(uint8_t *channel, uint8_t *status);


//функции, доступные для main
uint8_t ADC_SETUP_0(void);
uint8_t ADC_CALIBR(uint8_t meas_chn);
uint8_t ADC_change_filter_length(uint16_t new_length);

//инициализация регистров для работы с АЦП
void ADC_RG_INIT()
{
	uint32_t value = 0;

	value = 0;
	AD7124_Status.value = value;
	AD7124_Status.addr = 0x00;
	AD7124_Status.size = 1;
	AD7124_Status.rw = AD7124_R;	
	
	value = 0;
	value |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3) | AD7124_ADC_CTRL_REG_MODE(0x0) | AD7124_ADC_CTRL_REG_CLK_SEL(0x1) | AD7124_ADC_CTRL_REG_DATA_STATUS;
	AD7124_ADC_Control.value = value;
	AD7124_ADC_Control.addr = 0x01;
	AD7124_ADC_Control.size = 2;
	AD7124_ADC_Control.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Data.value = value;
	AD7124_Data.addr = 0x02;
	AD7124_Data.size = 3;
	AD7124_Data.rw = AD7124_R;	
	

	value = 0;
	//value |= AD7124_IO_CTRL1_REG_IOUT0(0x4) | AD7124_IO_CTRL1_REG_IOUT_CH0(0);
	value |= AD7124_IO_CTRL1_REG_IOUT0(ADC_IOUT_100uA) | AD7124_IO_CTRL1_REG_IOUT_CH0(0);
	AD7124_IOCon1.value = value;
	AD7124_IOCon1.addr = 0x03;
	AD7124_IOCon1.size = 3;
	AD7124_IOCon1.rw = AD7124_RW;	
	
	value = 0;
	AD7124_IOCon2.value = value;
	AD7124_IOCon2.addr = 0x04;
	AD7124_IOCon2.size = 2;
	AD7124_IOCon2.rw = AD7124_RW;	
	
	value = 0x2;
	AD7124_ID.value = value;
	AD7124_ID.addr = 0x05;
	AD7124_ID.size = 1;
	AD7124_ID.rw = AD7124_R;	
	
	value = 0;
	AD7124_Error.value = value;
	AD7124_Error.addr = 0x06;
	AD7124_Error.size = 3;
	AD7124_Error.rw = AD7124_R;	
	
	//value = 0x40;
	value = 0;
//	value |=  AD7124_ERR_REG_ADC_CAL_ERR | AD7124_ERR_REG_ADC_CONV_ERR| AD7124_ERR_REG_ADC_SAT_ERR |  
//	AD7124_ERR_REG_AINP_OV_ERR | AD7124_ERR_REG_AINP_UV_ERR | AD7124_ERR_REG_AINM_OV_ERR | AD7124_ERR_REG_AINM_UV_ERR | 
//	AD7124_ERR_REG_SPI_READ_ERR | AD7124_ERR_REG_SPI_WRITE_ERR;
	AD7124_Error_En.value = value;
	AD7124_Error_En.addr = 0x07;
	AD7124_Error_En.size = 3;
	AD7124_Error_En.rw = AD7124_RW;		
	
	value = 0;
	AD7124_Mclk_Count.value = value;
	AD7124_Mclk_Count.addr = 0x08;
	AD7124_Mclk_Count.size = 1;
	AD7124_Mclk_Count.rw = AD7124_R;	
	
	value = 0;
	value |= AD7124_CH_MAP_REG_CH_ENABLE | AD7124_CH_MAP_REG_SETUP(0x0) | AD7124_CH_MAP_REG_AINP(0x1) | AD7124_CH_MAP_REG_AINM(0x2);
	AD7124_Channel_0.value = value;
	AD7124_Channel_0.addr = 0x09;
	AD7124_Channel_0.size = 2;
	AD7124_Channel_0.rw = AD7124_RW;	
	
	value = 0;
	value |= AD7124_CH_MAP_REG_CH_ENABLE | AD7124_CH_MAP_REG_SETUP(0x1) | AD7124_CH_MAP_REG_AINP(0x3) | AD7124_CH_MAP_REG_AINM(0x4);
	AD7124_Channel_1.value = value;
	AD7124_Channel_1.addr = 0x0A;
	AD7124_Channel_1.size = 2;
	AD7124_Channel_1.rw = AD7124_RW;	
	
	value = 0;
	value |= AD7124_CH_MAP_REG_CH_ENABLE | AD7124_CH_MAP_REG_SETUP(0x2) | AD7124_CH_MAP_REG_AINP(0x5) | AD7124_CH_MAP_REG_AINM(0x6);
	AD7124_Channel_2.value = value;
	AD7124_Channel_2.addr = 0x0B;
	AD7124_Channel_2.size = 2;
	AD7124_Channel_2.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_3.value = value;
	AD7124_Channel_3.addr = 0x0C;
	AD7124_Channel_3.size = 2;
	AD7124_Channel_3.rw = AD7124_RW;	
		
	value = 0;
	AD7124_Channel_4.value = value;
	AD7124_Channel_4.addr = 0x0D;
	AD7124_Channel_4.size = 2;
	AD7124_Channel_4.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_5.value = value;
	AD7124_Channel_5.addr = 0x0E;
	AD7124_Channel_5.size = 2;
	AD7124_Channel_5.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_6.value = value;
	AD7124_Channel_6.addr = 0x0F;
	AD7124_Channel_6.size = 2;
	AD7124_Channel_6.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_7.value = value;
	AD7124_Channel_7.addr = 0x10;
	AD7124_Channel_7.size = 2;
	AD7124_Channel_7.rw = AD7124_RW;	
		
	value = 0;
	AD7124_Channel_8.value = value;
	AD7124_Channel_8.addr = 0x11;
	AD7124_Channel_8.size = 2;
	AD7124_Channel_8.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_9.value = value;
	AD7124_Channel_9.addr = 0x12;
	AD7124_Channel_9.size = 2;
	AD7124_Channel_9.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_10.value = value;
	AD7124_Channel_10.addr = 0x13;
	AD7124_Channel_10.size = 2;
	AD7124_Channel_10.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_11.value = value;
	AD7124_Channel_11.addr = 0x14;
	AD7124_Channel_11.size = 2;
	AD7124_Channel_11.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_12.value = value;
	AD7124_Channel_12.addr = 0x15;
	AD7124_Channel_12.size = 2;
	AD7124_Channel_12.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_13.value = value;
	AD7124_Channel_13.addr = 0x16;
	AD7124_Channel_13.size = 2;
	AD7124_Channel_13.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_14.value = value;
	AD7124_Channel_14.addr = 0x17;
	AD7124_Channel_14.size = 2;
	AD7124_Channel_14.rw = AD7124_RW;	
	
	value = 0;
	AD7124_Channel_15.value = value;
	AD7124_Channel_15.addr = 0x18;
	AD7124_Channel_15.size = 2;
	AD7124_Channel_15.rw = AD7124_RW;	
	
	
	value = 0;
	//value |= AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(0x0) | AD7124_CFG_REG_PGA(0x0);
		//ADC_GAIN_PGA_CHN0
	value |= AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(0x0) | AD7124_CFG_REG_PGA(ADC_GAIN_CHN0);
	AD7124_Config_0.value = value;
	AD7124_Config_0.addr = 0x19;
	AD7124_Config_0.size = 2;
	AD7124_Config_0.rw = AD7124_RW;	
	
	value = 0;
	//value |= AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(0x0) | AD7124_CFG_REG_PGA(0x0);
		//ADC_GAIN_PGA_CHN1
	value |= AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM | AD7124_CFG_REG_REF_SEL(0x0) | AD7124_CFG_REG_PGA(ADC_GAIN_CHN1);
	AD7124_Config_1.value = value;
	AD7124_Config_1.addr = 0x1A;
	AD7124_Config_1.size = 2;
	AD7124_Config_1.rw = AD7124_RW;	
	
	value = 0;
	value |= AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_REF_SEL(0x0) | AD7124_CFG_REG_PGA(0x0);
	AD7124_Config_2.value = value;
	AD7124_Config_2.addr = 0x1B;
	AD7124_Config_2.size = 2;
	AD7124_Config_2.rw = AD7124_RW;	
	
	value = 0x0860;
	AD7124_Config_3.value = value;
	AD7124_Config_3.addr = 0x1C;
	AD7124_Config_3.size = 2;
	AD7124_Config_3.rw = AD7124_RW;	
	
	value = 0x0860;
	AD7124_Config_4.value = value;
	AD7124_Config_4.addr = 0x1D;
	AD7124_Config_4.size = 2;
	AD7124_Config_4.rw = AD7124_RW;	
	
	value = 0x0860;
	AD7124_Config_5.value = value;
	AD7124_Config_5.addr = 0x1E;
	AD7124_Config_5.size = 2;
	AD7124_Config_5.rw = AD7124_RW;	
	
	value = 0x0860;
	AD7124_Config_6.value = value;
	AD7124_Config_6.addr = 0x1F;
	AD7124_Config_6.size = 2;
	AD7124_Config_6.rw = AD7124_RW;	
	
	value = 0x0860;
	AD7124_Config_7.value = value;
	AD7124_Config_7.addr = 0x20;
	AD7124_Config_7.size = 2;
	AD7124_Config_7.rw = AD7124_RW;	
	
	value = 0;
	value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
	AD7124_Filter_0.value = value;
	AD7124_Filter_0.addr = 0x21;
	AD7124_Filter_0.size = 3;
	AD7124_Filter_0.rw = AD7124_RW;	
	
	value = 0;
	value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
	AD7124_Filter_1.value = value;
	AD7124_Filter_1.addr = 0x22;
	AD7124_Filter_1.size = 3;
	AD7124_Filter_1.rw = AD7124_RW;	
	
	value = 0;
	value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
	AD7124_Filter_2.value = value;
	AD7124_Filter_2.addr = 0x23;
	AD7124_Filter_2.size = 3;
	AD7124_Filter_2.rw = AD7124_RW;	
	
	value = 0x060180;
	AD7124_Filter_3.value = value;
	AD7124_Filter_3.addr = 0x24;
	AD7124_Filter_3.size = 3;
	AD7124_Filter_3.rw = AD7124_RW;	
	
	value = 0x060180;
	AD7124_Filter_4.value = value;
	AD7124_Filter_4.addr = 0x25;
	AD7124_Filter_4.size = 3;
	AD7124_Filter_4.rw = AD7124_RW;	
	
	value = 0x060180;
	AD7124_Filter_5.value = value;
	AD7124_Filter_5.addr = 0x26;
	AD7124_Filter_5.size = 3;
	AD7124_Filter_5.rw = AD7124_RW;	
	
	value = 0x060180;
	AD7124_Filter_6.value = value;
	AD7124_Filter_6.addr = 0x27;
	AD7124_Filter_6.size = 3;
	AD7124_Filter_6.rw = AD7124_RW;	
	
	value = 0x060180;
	AD7124_Filter_7.value = value;
	AD7124_Filter_7.addr = 0x28;
	AD7124_Filter_7.size = 3;
	AD7124_Filter_7.rw = AD7124_RW;	



	value = AD7124_OFFSET0;
	AD7124_Offset_0.value = value;
	AD7124_Offset_0.addr = 0x29;
	AD7124_Offset_0.size = 3;
	AD7124_Offset_0.rw = AD7124_RW;	
	
	value =  AD7124_OFFSET1;
	AD7124_Offset_1.value = value;
	AD7124_Offset_1.addr = 0x2A;
	AD7124_Offset_1.size = 3;
	AD7124_Offset_1.rw = AD7124_RW;
	
	value =  AD7124_OFFSET2;
	AD7124_Offset_2.value = value;
	AD7124_Offset_2.addr = 0x2B;
	AD7124_Offset_2.size = 3;
	AD7124_Offset_2.rw = AD7124_RW;
	
	value = 0x800000;
	AD7124_Offset_3.value = value;
	AD7124_Offset_3.addr = 0x2C;
	AD7124_Offset_3.size = 3;
	AD7124_Offset_3.rw = AD7124_RW;
	
	value = 0x800000;
	AD7124_Offset_4.value = value;
	AD7124_Offset_4.addr = 0x2D;
	AD7124_Offset_4.size = 3;
	AD7124_Offset_4.rw = AD7124_RW;
	
	value = 0x800000;
	AD7124_Offset_5.value = value;
	AD7124_Offset_5.addr = 0x2E;
	AD7124_Offset_5.size = 3;
	AD7124_Offset_5.rw = AD7124_RW;
	
	value = 0x800000;
	AD7124_Offset_6.value = value;
	AD7124_Offset_6.addr = 0x2F;
	AD7124_Offset_6.size = 3;
	AD7124_Offset_6.rw = AD7124_RW;
	
	value = 0x800000;
	AD7124_Offset_7.value = value;
	AD7124_Offset_7.addr = 0x30;
	AD7124_Offset_7.size = 3;
	AD7124_Offset_7.rw = AD7124_RW;


	value = AD7124_GAIN0;
	AD7124_Gain_0.value = value;
	AD7124_Gain_0.addr = 0x31;
	AD7124_Gain_0.size = 3;
	AD7124_Gain_0.rw = AD7124_RW;
	
	value = AD7124_GAIN1;
	AD7124_Gain_1.value = value;
	AD7124_Gain_1.addr = 0x32;
	AD7124_Gain_1.size = 3;
	AD7124_Gain_1.rw = AD7124_RW;
	
	value = AD7124_GAIN2;
	AD7124_Gain_2.value = value;
	AD7124_Gain_2.addr = 0x33;
	AD7124_Gain_2.size = 3;
	AD7124_Gain_2.rw = AD7124_RW;
	
	value = 0x500000;
	AD7124_Gain_3.value = value;
	AD7124_Gain_3.addr = 0x34;
	AD7124_Gain_3.size = 3;
	AD7124_Gain_3.rw = AD7124_RW;
	
	value = 0x500000;
	AD7124_Gain_4.value = value;
	AD7124_Gain_4.addr = 0x35;
	AD7124_Gain_4.size = 3;
	AD7124_Gain_4.rw = AD7124_RW;
	
	value = 0x500000;
	AD7124_Gain_5.value = value;
	AD7124_Gain_5.addr = 0x36;
	AD7124_Gain_5.size = 3;
	AD7124_Gain_5.rw = AD7124_RW;
	
	value = 0x500000;
	AD7124_Gain_6.value = value;
	AD7124_Gain_6.addr = 0x37;
	AD7124_Gain_6.size = 3;
	AD7124_Gain_6.rw = AD7124_RW;
	
	value = 0x500000;
	AD7124_Gain_7.value = value;
	AD7124_Gain_7.addr = 0x38;
	AD7124_Gain_7.size = 3;
	AD7124_Gain_7.rw = AD7124_RW;


	ch_thrm.factor = 2.5/(16777216.0 * 16.0);
	ch_thrm.offset = 0.0;
	ch_curr.factor = 2.5/(16777216.0 * 16.0) ;
	ch_curr.offset = 0.0;
	ch_hv.factor = (2.5/16777216.0);
	ch_hv.offset = 0.0;

}


//сброс интерефейса с помощью специальной последовательности битов
uint8_t ADC_SOFT_RESET(void)
{
	HAL_SPI_StateTypeDef SPI_status = HAL_SPI_STATE_READY;
	SPI_status = HAL_SPI_GetState(&hspi1);
	HAL_StatusTypeDef SPI_TRX_status = HAL_OK;
	
	uint8_t reset_com[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	//digital interface reset
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);
	
	SPI_TRX_status = HAL_SPI_Transmit(&hspi1, &reset_com[0], sizeof(reset_com), timeout_spi);
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);
	if (SPI_response(SPI_TRX_status)!=1) return 0;

	SPI_status = HAL_SPI_GetState(&hspi1);
	if(SPI_status==HAL_SPI_STATE_READY)
		return 1;
	else
		return 0;
}

void ADC_HARD_RESET(void)
{
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		HAL_GPIO_WritePin(GPIOD, SPI1_SYNC_ADC_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOD, SPI1_SYNC_ADC_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
}


//запись в регистр АЦП нового значения data
uint8_t ADC_write(REG_STRUCT *ptr_rg, uint32_t data)
{
	HAL_GPIO_WritePin(GPIOC, TP_SPI1_Pin, GPIO_PIN_SET);
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	uint32_t data_copy = data;
	uint8_t buff[4]= {0,0,0,0};
	uint32_t *ptr_data = &data_copy;
	uint8_t data_byte = 0;
	uint8_t size_rg = ptr_rg->size;
	uint8_t comm_reg_val = 0;
	uint32_t mask = 0x0;
	uint8_t internal_addr = (ptr_rg->addr)&0x3F;
	
	if (ptr_rg->rw == AD7124_R)
		return 0;

		HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);
		comm_reg_val = 0;
		comm_reg_val |= AD7124_COMM_REG_WEN|AD7124_COMM_REG_WR|AD7124_COMM_REG_RA(internal_addr);
		HAL_SPI_Transmit(&hspi1, &comm_reg_val, sizeof(uint8_t), timeout_spi);//направляем адрес
		for (uint32_t cntr = size_rg; cntr >0; cntr-- )//байтовая передача
			{
				data_byte = (data_copy & (0xFF<<8*(cntr-1))) >> 8*(cntr-1);
				HAL_SPI_Transmit(&hspi1, &data_byte, sizeof(uint8_t), timeout_spi);
			}
		HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);
	
	//если всё успешно, то синхронизируем значение в структуре
	mask = (1 << (ptr_rg->size*8)) - 1;
	ptr_rg->value = data_copy & mask;
	HAL_GPIO_WritePin(GPIOC, TP_SPI1_Pin, GPIO_PIN_RESET);
	return 1;//OK
}


uint8_t ADC_read (REG_STRUCT *ptr_rg, uint32_t *data)
{
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	uint8_t size_rg = ptr_rg->size;
	uint32_t mask = (1 << (size_rg*8)) - 1;
	*data =  0x0;

	uint8_t rx_buff[size_rg];
	uint8_t HL_SQNC[size_rg];
	uint8_t *ptr_rx_buff = rx_buff;
	uint8_t *ptr_HL_SQNC = HL_SQNC;
	uint8_t comm_reg_val = 0;

	uint8_t internal_addr = (ptr_rg->addr)&0x3F;
	
	for(uint32_t i = 0 ; i< size_rg; i++)
	{
	 *ptr_HL_SQNC = 0xFF;
		ptr_HL_SQNC++;
	}
	ptr_HL_SQNC = &HL_SQNC[0];
	
		HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);
		comm_reg_val = 0;
		comm_reg_val |= AD7124_COMM_REG_WEN|AD7124_COMM_REG_RD|AD7124_COMM_REG_RA(internal_addr);
		HAL_SPI_Transmit(&hspi1, &comm_reg_val, sizeof(uint8_t), timeout_spi);//направляем адрес
		HAL_SPI_TransmitReceive(&hspi1, ptr_HL_SQNC, ptr_rx_buff, size_rg * sizeof(uint8_t), timeout_spi);
		HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);

		
	//если всё успешно, то записываем значение в структуру
	uint32_t temp = 0;
	for(uint32_t i = size_rg; i > 0; i--)
	{
		temp |= (*ptr_rx_buff) << ((i-1)*8);
		ptr_rx_buff ++;
	}
	mask = (1 << (ptr_rg->size*8)) - 1;
	ptr_rg->value = temp & mask;

	*data = ptr_rg->value;
	return 1;//OK
}

//функция проверки статуса интерфейса для контроля над ошибками
uint8_t SPI_response(HAL_StatusTypeDef SPI_TRX_Status)
{
	HAL_SPI_StateTypeDef SPI_Status = HAL_SPI_STATE_READY;
	
	if(SPI_TRX_Status!=HAL_OK)
		{
			if((SPI_TRX_Status == HAL_TIMEOUT)||(SPI_TRX_Status == HAL_ERROR))
				return 0;
			
			if(SPI_TRX_Status == HAL_BUSY) 
				for(uint32_t i = 0; i < waiting_cycles; i++) 
				{
					SPI_Status = HAL_SPI_GetState(&hspi1);
					if (SPI_Status == HAL_SPI_STATE_READY) 
						return 1;
				}
				
			if((SPI_Status==HAL_SPI_STATE_ERROR)||(SPI_Status==HAL_SPI_STATE_BUSY) ) 
				return 0;
		}
		
	return 1;//OK
}

//настройка запись регистров, инициализированных функцией ADC_RG_INIT 
uint8_t ADC_SETUP_0(void)
{
	uint32_t value;
	
	uint32_t ID_query = 0;
	ptr_struct = &AD7124_ID;
	
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	
	ADC_HARD_RESET();
	HAL_Delay(1);
	ADC_SOFT_RESET();
	HAL_Delay(1);
	
	ADC_read(ptr_struct, &ID_query);
	HAL_Delay(1);
	if ((ID_query==0x0)||(ID_query==0xFF))
	{
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
		Error_Handler();
		return 0;
	}
	
	HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);

	//channel 0
	ADC_write(&AD7124_Channel_0, AD7124_Channel_0.value);
	ADC_write(&AD7124_Config_0, AD7124_Config_0.value);
	ADC_write(&AD7124_Filter_0, AD7124_Filter_0.value);
	ADC_write(&AD7124_Offset_0, AD7124_Offset_0.value);
	ADC_write(&AD7124_Gain_0, AD7124_Gain_0.value);
	
	//channel 1
	ADC_write(&AD7124_Channel_1, AD7124_Channel_1.value);
	ADC_write(&AD7124_Config_1, AD7124_Config_1.value);
	ADC_write(&AD7124_Filter_1, AD7124_Filter_1.value);
	ADC_write(&AD7124_Offset_1, AD7124_Offset_1.value);
	ADC_write(&AD7124_Gain_1, AD7124_Gain_1.value);
	
	//channel 2
	ADC_write(&AD7124_Channel_2, AD7124_Channel_2.value);
	ADC_write(&AD7124_Config_2, AD7124_Config_2.value);
	ADC_write(&AD7124_Filter_2, AD7124_Filter_2.value);
	ADC_write(&AD7124_Offset_2, AD7124_Offset_2.value);
	ADC_write(&AD7124_Gain_2, AD7124_Gain_2.value);
	
	//control reg
	ADC_write(&AD7124_ADC_Control, AD7124_ADC_Control.value);
	
	//IOCon1 reg
	ADC_write(&AD7124_IOCon1, AD7124_IOCon1.value);
	
	//Error_en_reg
	ADC_write(&AD7124_Error_En,  AD7124_Error_En.value);
	
	return 1;
}

uint8_t ADC_CALIBR(uint8_t meas_chn)
{
	uint32_t value;
	uint32_t mask;
	uint32_t code;
	volatile uint32_t f;
	meas_chn &= 0x3; 
	uint32_t filter_calibrate = 0x3FF;
	GPIO_PinState RDY_LEVEL_STATE = GPIO_PIN_SET;
	
	
	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);
	switch(meas_chn)
	{
		//*************калибровка канала 0 - измерение напряжения на платиновом термодатчике
		case 0:
		{
			//оставляем включенным только выбранный измерительный канал
			mask = 0;
			mask |= AD7124_CH_MAP_REG_CH_ENABLE;
			mask = ~mask;
						
			value = AD7124_Channel_1.value & mask;
			ADC_write(&AD7124_Channel_1, value);
			
			value = AD7124_Channel_2.value & mask;
			ADC_write(&AD7124_Channel_2, value);
			
			//по рекомендации даташита, инициализируем OFFSET регистр
			ADC_write(&AD7124_Offset_0, AD7124_Offset_0.value);
			
			//переключаем режим работы на low power
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			ADC_write(&AD7124_ADC_Control, value);
			
			//увеличиваем длину фильтра до максимального 
			filter_calibrate = 0x3FF;
			value = 0;
			value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(filter_calibrate);
			ADC_write(& AD7124_Filter_0, value);
			
			mask = 0;
			mask |= AD7124_CFG_REG_PGA(0x7);
			if ( (AD7124_Config_0.value & mask) != 0 )
			{
					//для GAIN = 1 FULL SCALE internal calibration не проводится
					//включаем режим full scale calibrate
					mask = 0;
					mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
					mask = ~mask;
					value = AD7124_ADC_Control.value & mask;
					value |= AD7124_ADC_CTRL_REG_MODE(0x6);
					ADC_write(&AD7124_ADC_Control, value);

					HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);		
					f = 1;
					RDY_LEVEL_STATE = GPIO_PIN_SET;
					while (f) 
					{
						RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
						if (RDY_LEVEL_STATE == GPIO_PIN_RESET)
						{
							f = 0;
						}
						
					}
	

					ptr_struct = &AD7124_Gain_0;
					value = 0;
					ADC_read(ptr_struct, &value);
					AD7124_Gain_0.value = value;
					AD7124_GAIN0 = value;
			}
			
			//zero_scale проводится для всех коэффициентов
			//включаем режим zero scale calibrate
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			value |= AD7124_ADC_CTRL_REG_MODE(0x5);
			ADC_write(&AD7124_ADC_Control, value);
			
			
			HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);		
			f = 1;
			RDY_LEVEL_STATE = GPIO_PIN_SET;
			while (f) 
			{
				RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
				if (RDY_LEVEL_STATE == GPIO_PIN_RESET)
				{
					f = 0;
				}
			}
		
			ptr_struct = &AD7124_Offset_0;
			value = 0;
			ADC_read(ptr_struct, &value);
			AD7124_Offset_0.value = value;
			AD7124_OFFSET0 = value;

			//переключаем режим работы на continuous convertion
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			ADC_write(&AD7124_ADC_Control, value);
									
			//восстанавливаем работу измерительных каналов
			mask = 0;
			mask |= AD7124_CH_MAP_REG_CH_ENABLE;
			value = AD7124_Channel_1.value | mask;
			ADC_write(&AD7124_Channel_1, value);
			
			value = AD7124_Channel_2.value | mask;
			ADC_write(&AD7124_Channel_2, value);
			
			//переключаем режим работы на full power
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3);
			value = AD7124_ADC_Control.value | mask;
			ADC_write(&AD7124_ADC_Control, value);
			
			//возвращаем прежнее значение фильтра
			value = 0;
			value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
			ADC_write(& AD7124_Filter_0, value);

			break;
		}
		//*************калибровка канала 1 - измерение напряжения на токоизмерительном резисторе
		case 1:
		{
			mask = 0;
			mask |= AD7124_CH_MAP_REG_CH_ENABLE;
			mask = ~mask;
						
			value = AD7124_Channel_0.value & mask;
			ADC_write(&AD7124_Channel_0, value);
			
			value = AD7124_Channel_2.value & mask;
			ADC_write(&AD7124_Channel_2, value);
						
			ADC_write(&AD7124_Offset_1, AD7124_Offset_1.value);
			
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			ADC_write(&AD7124_ADC_Control, value);
			
			filter_calibrate = 0x3FF;
			value = 0;
			value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(filter_calibrate);
			ADC_write(& AD7124_Filter_1, value);
			
			mask = 0;
			mask |= AD7124_CFG_REG_PGA(0x7);
			if ( (AD7124_Config_1.value & mask) != 0 )
			{
					//для GAIN = 1 FULL SCALE internal calibration не проводится
					//включаем режим full scale calibrate
					mask = 0;
					mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
					mask = ~mask;
					value = AD7124_ADC_Control.value & mask;
					value |= AD7124_ADC_CTRL_REG_MODE(0x6);
					ADC_write(&AD7124_ADC_Control, value);

					HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);			
					f = 1;
					RDY_LEVEL_STATE = GPIO_PIN_SET;
					while (f) 
					{
						RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
						if (RDY_LEVEL_STATE == GPIO_PIN_RESET)
						{
							//EXTI9_5_IRQHandler();
							f = 0;
						}
						
					}
					ptr_struct = &AD7124_Gain_1;
					value = 0;
					ADC_read(ptr_struct, &value);
					AD7124_Gain_1.value = value;
					AD7124_GAIN1 = value;
			}
			
			//zero_scale проводится для всех коэффициентов
			//включаем режим zero scale calibrate
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			value |= AD7124_ADC_CTRL_REG_MODE(0x5);
			ADC_write(&AD7124_ADC_Control, value);
			
			
			HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);		
			f = 1;
			RDY_LEVEL_STATE = GPIO_PIN_SET;
			while (f) 
			{
				RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
				if (RDY_LEVEL_STATE == GPIO_PIN_RESET)
				{
					//EXTI9_5_IRQHandler();
					f = 0;
				}
				
			}

			
			ptr_struct = &AD7124_Offset_1;
			value = 0;
			ADC_read(ptr_struct, &value);
			AD7124_Offset_1.value = value;
			AD7124_OFFSET1 = value;

			//переключаем режим работы на continuous convertion
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			ADC_write(&AD7124_ADC_Control, value);
									
			//восстанавливаем работу измерительных каналов
			mask = 0;
			mask |= AD7124_CH_MAP_REG_CH_ENABLE;
			value = AD7124_Channel_0.value | mask;
			ADC_write(&AD7124_Channel_0, value);
			
			value = AD7124_Channel_2.value | mask;
			ADC_write(&AD7124_Channel_2, value);
			
			//переключаем режим работы на full power
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3);
			value = AD7124_ADC_Control.value | mask;
			ADC_write(&AD7124_ADC_Control, value);
			
			//возвращаем прежнее значение фильтра
			value = 0;
			value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
			ADC_write(& AD7124_Filter_1, value);
			break;
		}
		
		//*************калибровка канала 2 - измерение напряжения на делителе высокого напряжения смещения фотодиода
		case 2:
		{
			mask = 0;
			mask |= AD7124_CH_MAP_REG_CH_ENABLE;
			mask = ~mask;
						
			value = AD7124_Channel_0.value & mask;
			ADC_write(&AD7124_Channel_0, value);
			
			value = AD7124_Channel_1.value & mask;
			ADC_write(&AD7124_Channel_1, value);
						
			ADC_write(&AD7124_Offset_2, AD7124_Offset_2.value);
			
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			ADC_write(&AD7124_ADC_Control, value);
			
			filter_calibrate = 0x3FF;
			value = 0;
			value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(filter_calibrate);
			ADC_write(& AD7124_Filter_2, value);
			
			mask = 0;
			mask |= AD7124_CFG_REG_PGA(0x7);
			if ( (AD7124_Config_2.value & mask) != 0 )
			{
					//для GAIN = 1 FULL SCALE internal calibration не проводится
					//включаем режим full scale calibrate
					mask = 0;
					mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
					mask = ~mask;
					value = AD7124_ADC_Control.value & mask;
					value |= AD7124_ADC_CTRL_REG_MODE(0x6);
					ADC_write(&AD7124_ADC_Control, value);

					HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);			
					f = 1;
					RDY_LEVEL_STATE = GPIO_PIN_SET;
					while (f) 
					{
						RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
						if (RDY_LEVEL_STATE == GPIO_PIN_RESET)
						{
							//EXTI9_5_IRQHandler();
							f = 0;
						}
						
					}

					ptr_struct = &AD7124_Gain_2;
					value = 0;
					ADC_read(ptr_struct, &value);
					AD7124_Gain_2.value = value;
					AD7124_GAIN2 = value;
			}
			
			//zero_scale проводится для всех коэффициентов
			//включаем режим zero scale calibrate
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			value |= AD7124_ADC_CTRL_REG_MODE(0x5);
			ADC_write(&AD7124_ADC_Control, value);
			
			
			HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);		
			f = 1;
			RDY_LEVEL_STATE = GPIO_PIN_SET;
			while (f) 
			{
				RDY_LEVEL_STATE = HAL_GPIO_ReadPin (GPIOA, SPI1_RDY_LVL_Pin);
				if (RDY_LEVEL_STATE == GPIO_PIN_RESET)
				{
					//EXTI9_5_IRQHandler();
					f = 0;
				}
				
			}
			
			ptr_struct = &AD7124_Offset_2;
			value = 0;
			ADC_read(ptr_struct, &value);
			AD7124_Offset_2.value = value;
			AD7124_OFFSET2 = value;

			//переключаем режим работы на continuous convertion
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_MODE(0xF);
			mask = ~mask;
			value = AD7124_ADC_Control.value & mask;
			ADC_write(&AD7124_ADC_Control, value);
									
			//восстанавливаем работу измерительных каналов
			mask = 0;
			mask |= AD7124_CH_MAP_REG_CH_ENABLE;
			value = AD7124_Channel_0.value | mask;
			ADC_write(&AD7124_Channel_0, value);
			
			value = AD7124_Channel_1.value | mask;
			ADC_write(&AD7124_Channel_1, value);
			
			//переключаем режим работы на full power
			mask = 0;
			mask |= AD7124_ADC_CTRL_REG_POWER_MODE(0x3);
			value = AD7124_ADC_Control.value | mask;
			ADC_write(&AD7124_ADC_Control, value);
			
			//возвращаем прежнее значение фильтра
			value = 0;
			value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
			ADC_write(&AD7124_Filter_2, value);
			break;
		}
			
		default:
		{
			
			break;
		}
		
		
	}
	

	HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);
	return 1;
}


uint32_t ADC_read_DATA(uint8_t *channel, uint8_t *status)
{
		//HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_RESET);
	
		uint32_t tmp = 0;
		uint8_t data_status = 0;
		uint8_t adc_chn_sel = 0;
		uint8_t tx8 = 0;
		uint8_t tx24[3] = {0};

		uint8_t rx8 = {0};
		uint8_t rx24[3] = {0};
		
		tx8 = 0x42;
		rx24[0] = 0x00;
		rx24[1] = 0x00;
		rx24[2] = 0x00;
		tx24[0] = 0xFF;
		tx24[1] = 0xFF;
		tx24[2] = 0xFF;
		HAL_SPI_Transmit(&hspi1, &tx8, sizeof(uint8_t), 0x1000);//query data
		HAL_SPI_TransmitReceive(&hspi1, &tx24[0], &rx24[0], 3*sizeof(uint8_t), 0x1000);//reading DATA
		tx8 = 0;
		rx8 = 0;
		HAL_SPI_TransmitReceive(&hspi1, &tx8, &rx8, sizeof(uint8_t), 0x1000);//reading DATA_STATUS
		//__HAL_GPIO_EXTI_CLEAR_FLAG( SPI1_RDY_ADC_Pin);
		//HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
		
		adc_chn_sel = rx8 & 0x0F;
		data_status = rx8 & 0x40; 

		//в любом случаем заполняем буфер
		tmp = 0;
		tmp |= rx24[2];
		tmp |= (rx24[1]<<8);
		tmp |= (rx24[0]<<16);
		Put(tmp, &_fifos[ adc_chn_sel ]);
		
		
		
//		if (data_status == 0)
//		{
//			tmp = 0;
//			tmp |= rx24[2];
//			tmp |= (rx24[1]<<8);
//			tmp |= (rx24[0]<<16);
//			Put(tmp, &_fifos[ adc_chn_sel ]);
//		}
//		else
//		{
//			return 0;//ошибка АЦП
//		}
		
		*channel = adc_chn_sel;//возвращаем номер канала
		*status = data_status;//возращаем статус АЦП
		return tmp;

		//HAL_GPIO_WritePin(GPIOD, SPI1_CS_ADC_Pin, GPIO_PIN_SET);
}

uint8_t ADC_change_filter_length(uint16_t new_length)
{
	uint32_t value = 0;
	//value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | AD7124_FILT_REG_FS(ADC_FILT_LEN);
	if((new_length > 0) && (new_length <= 2047))
		value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | 	AD7124_FILT_REG_FS(new_length);
	else
		return 0;
		//value |= AD7124_FILT_REG_FILTER(0x0) | AD7124_FILT_REG_SINGLE_CYCLE | AD7124_FILT_REG_POST_FILTER(0x0) | 	AD7124_FILT_REG_FS(ADC_FILT_LEN);
	
	ADC_write(&AD7124_Filter_0, value);
	ADC_write(&AD7124_Filter_1, value);
	ADC_write(&AD7124_Filter_2, value);
	return 1;
}

