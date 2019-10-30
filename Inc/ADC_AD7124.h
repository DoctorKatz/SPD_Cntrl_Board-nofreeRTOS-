#include "stm32f4xx_hal.h"
#include "AD7124_regs.h"


#define ADC_ADDR_MAX 0x39//максимальный адрес регистра АЦП(адресация внутренняя) + 1 (для циклов)

#define ADC_REF_RES 1000//ohm номинал прецизионного резистора

#define ADC_FILT_LEN 24// длина фильтра по умолчанию

//коды значений опорного источника тока АЦП
#define ADC_IOUT_OFF 		0x0
#define ADC_IOUT_50uA 	0x1
#define ADC_IOUT_100uA 	0x2
#define ADC_IOUT_250uA 	0x3
#define ADC_IOUT_500uA 	0x4
#define ADC_IOUT_750uA 	0x5
#define ADC_IOUT_1000uA 0x6


/*! ADC registers list*/
typedef struct 
{
	uint32_t ADC_Status;//read only
	uint32_t ADC_ADC_Control;
	uint32_t ADC_Data;
	uint32_t ADC_IOCon1;
	uint32_t ADC_IOCon2;
	uint32_t ADC_IDp;
	uint32_t ADC_Error;
	uint32_t ADC_Error_En;
	uint32_t ADC_Mclk_Count;
	uint32_t ADC_Channel_0;
	uint32_t ADC_Channel_1;
	uint32_t ADC_Channel_2;
	uint32_t ADC_Channel_3;
	uint32_t ADC_Channel_4;
	uint32_t ADC_Channel_5;
	uint32_t ADC_Channel_6;
	uint32_t ADC_Channel_7;
	uint32_t ADC_Channel_8;
	uint32_t ADC_Channel_9;
	uint32_t ADC_Channel_10;
	uint32_t ADC_Channel_11;
	uint32_t ADC_Channel_12;
	uint32_t ADC_Channel_13;
	uint32_t ADC_Channel_14;
	uint32_t ADC_Channel_15;	
	uint32_t ADC_Config_0;
	uint32_t ADC_Config_1;
	uint32_t ADC_Config_2;
	uint32_t ADC_Config_3;
	uint32_t ADC_Config_4;
	uint32_t ADC_Config_5;
	uint32_t ADC_Config_6;
	uint32_t ADC_Config_7;
	uint32_t ADC_Filter_0;
	uint32_t ADC_Filter_1;
	uint32_t ADC_Filter_2;
	uint32_t ADC_Filter_3;
	uint32_t ADC_Filter_4;
	uint32_t ADC_Filter_5;
	uint32_t ADC_Filter_6;
	uint32_t ADC_Filter_7;
	uint32_t ADC_Offset_0;
	uint32_t ADC_Offset_1;
	uint32_t ADC_Offset_2;
	uint32_t ADC_Offset_3;
	uint32_t ADC_Offset_4;
	uint32_t ADC_Offset_5;
	uint32_t ADC_Offset_6;
	uint32_t ADC_Offset_7;
	uint32_t ADC_Gain_0;
	uint32_t ADC_Gain_1;
	uint32_t ADC_Gain_2;
	uint32_t ADC_Gain_3;
	uint32_t ADC_Gain_4;
	uint32_t ADC_Gain_5;
	uint32_t ADC_Gain_6;
	uint32_t ADC_Gain_7;
	
	uint8_t ADC_REG_ADDR;
	uint32_t DATA_CHN0;
	uint32_t DATA_CHN1;
	uint32_t DATA_CHN2;
	
} regs_table;


typedef struct
{
	uint32_t ADC_Offset_CLBR_0;
	uint32_t ADC_Offset_CLBR_1;
	uint32_t ADC_Offset_CLBR_2;
	uint32_t ADC_Offset_CLBR_3;
	uint32_t ADC_Offset_CLBR_4;
	uint32_t ADC_Offset_CLBR_5;
	uint32_t ADC_Offset_CLBR_6;
	uint32_t ADC_Offset_CLBR_7;
	uint32_t ADC_Gain_CLBR_0;
	uint32_t ADC_Gain_CLBR_1;
	uint32_t ADC_Gain_CLBR_2;
	uint32_t ADC_Gain_CLBR_3;
	uint32_t ADC_Gain_CLBR_4;
	uint32_t ADC_Gain_CLBR_5;
	uint32_t ADC_Gain_CLBR_6;
	uint32_t ADC_Gain_CLBR_7;
} ADC_CALIBRATE;//регистры хранения калибровки

typedef struct
{
		float factor;
		float offset;
} line_coeff;



