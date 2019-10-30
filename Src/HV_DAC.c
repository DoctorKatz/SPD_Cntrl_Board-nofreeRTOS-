#include "main.h"
#include "stm32f4xx_hal.h"
#include "SPD_controller.h"
#include "usb_device.h"

#define MAX_NUM_POINTS 10U//максимальное количество точек для калибровки

uint8_t DAC_EXT_write(uint16_t data);//DAC8411 external DAC 16 bit
uint8_t DAC_INT_write(uint8_t data);//MAX1932 internal DAC 8 bit
void DAC_EXT_init(uint16_t data);
uint8_t DAC_EXT_calib(void);
uint8_t HV_code_to_value (uint16_t inp_code, float *outp_volt);
uint8_t HV_value_to_code (float inp_volt_val, uint16_t *outp_code);
uint8_t HV_calib_res (float inp_adc_value, float *outp_value);

uint8_t HV_value_set_outp (float *inp_value);
uint8_t HV_code_set_outp (uint16_t *inp_code);

extern uint8_t SDADC_conv (uint8_t chn, uint32_t *readed_data );
extern uint8_t ADC_change_filter_length(uint16_t new_length);
extern void tim3_delay(void);
extern float abs_value(uint8_t *sign, float value);
extern uint8_t calc_OLS(float *inp_x, float *inp_y, uint16_t size, double *outp_a, double *outp_b, float *outp_eps);//Ordinary Least Squares

//непосредственно таблица с данными
extern uint16_t code_tec[table_size];//управляющий код
extern uint8_t valid_tec[table_size];//флаг валидности
extern float value_tec[table_size];//значение напряжения в единицах СИ
extern float value_eps[table_size]; //
extern float code_float[ table_size ]; // 
 
extern DET_control DET;
extern SPD_regs SPD;
extern HV_src HV;
extern float floats[];
extern float floats_old[];
extern uint8_t sign;

float voltmeter_value[MAX_NUM_POINTS];//значения точек по калибр. вольтметру

struct
{
	double C0;
	double C1;
	double B0;
	double B1;
	double alpha0;
	double alpha1;
	double beta0;
} HV_coeff;




uint8_t DAC_EXT_write(uint16_t data)
{
	if(	DET.IsPowerOn == 0)
		return 0;
	
	uint32_t mask = 0;
	uint32_t value = 0;
	uint32_t temp = 0;
	
	HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_RESET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET);//AD9912
	
	tim3_delay();
	
	//формируем 24 разрядное слово для отправки
	temp = (uint32_t)data;
	temp = temp << 6;
	temp &= 0x3FFFC0;
	
	for(uint8_t j = 0 ; j < 24; j++)
	{
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_SET);
		
		tim3_delay();
		mask = ( 1 << (23 - j) );
		value = temp;
		value &= mask;
		if( value == 0 )
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_SET);
		
		tim3_delay();
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
		tim3_delay();
	}
	
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET);//AD9912
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	
	return 1;
}

uint8_t DAC_INT_write(uint8_t data)
{
	
	if(	DET.IsPowerOn == 0)
		return 0;
		
	uint32_t mask = 0;
	uint32_t value = 0;
	uint32_t temp = 0;
	
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_RESET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET);//AD9912	
	
	tim3_delay();
	
	//формируем 8 разрядное слово для отправки
	temp = (uint32_t)data;
	//temp = temp << 6;
	//temp &= 0x3FFFC0;
	for(uint8_t j = 0 ; j < 8; j++)
	{
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
		tim3_delay();
		mask = ( 1 << (7 - j) );
		value = temp;
		value &= mask;
		if( value == 0 )
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_SET);
		
		tim3_delay();
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_SET);
		tim3_delay();
	}
	
	
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET);//AD9912
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
	
	return 1;
}


void DAC_EXT_init(uint16_t data)
{
	if(DET.IsPowerOn == 0)
	{
		HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
		DET.IsPowerOn = 1;
		SPD.SPD_status_reg |= SPD_DET_pwr_flag;
		HAL_Delay(500);
	}
	DAC_EXT_write(data);
	HAL_Delay(50);
}

uint8_t DAC_EXT_calib(void)
{
	if (DET.IsPowerOn == 0)
		return 0;
	
	//ADC_change_filter_length(uint16_t new_length);
	//используем имеющийся свободный массив данных
	for(uint16_t ii = 0 ; ii < table_size ; ii++)
	{
		code_tec[ii] = 0;
		valid_tec[ii] = 0 ;
		value_tec[ii] = (float)0.0 ;
		value_eps[ii] = (float)0.0 ;
	}
	
	for(uint16_t ii = 0; ii < MAX_NUM_POINTS ; ii++ )
	{
		voltmeter_value[ii] = 0.0;
		code_float[ii] = 0.0;
	}
	
	uint16_t num_points = MAX_NUM_POINTS ;
	uint8_t conv_resp = 0;
	uint8_t SDADC_chn = 2;//канал измерения HV
	uint32_t readed_data = 0;
	uint16_t code_start = 0x3FBB;
	uint16_t code_stop = 0x0100;
	uint16_t code_step = (uint16_t)( (code_start - code_stop) / num_points ) ;
	uint16_t code_curr = 0x3FBB;
	uint16_t volt_setup_delay = 10 ;
	//float value_step = (float)( (HV.HV_max - HV.HV_min) / num_points ) ;
	float y1 = 0.0, y0 = 0.0, delta_y;
	
	DAC_EXT_write(code_start);
	HAL_Delay(500);
	for(uint16_t ii = 0 ; ii < num_points ; ii++)
	{
		HV.code = code_start - code_step * ii ;
		DAC_EXT_write(HV.code);
		
		//цикл проверки установившегося режима
		y1 = 0.0;
		y0 = 0.0;
		for(uint16_t jj = 0; jj < volt_setup_delay; jj++ )
		{
			HAL_Delay(200);
			conv_resp = 0;
			while(conv_resp == 0)
				conv_resp = SDADC_conv (SDADC_chn, &readed_data );
		
			y1 = HV.HV_Kadc * floats[SDADC_chn] ;
			delta_y = abs_value( &sign, (y1 - y0) );
			if(delta_y <= HV.HV_eps )
			{
				y1 = 0;
				float Kc = HV.HV_Kadc / (float)8.0;
				for(uint16_t rr = 0; rr < 8; rr++)
				{
					HAL_Delay(1);
					conv_resp = 0;
					while(conv_resp == 0)
						conv_resp = SDADC_conv (SDADC_chn, &readed_data );
					y1 += Kc * floats[SDADC_chn] ;
				}
				valid_tec[ii] = 0x1;
				value_tec[ii] = y1;
				code_tec[ii] = (uint16_t)HV.code ;
				code_float[ii] = (float)HV.code ;
				break;
			}
			y0 = y1;
		}
		
		if((valid_tec[ii] == 0) && (code_float[ii] == 0))//если режим так и не установился, то выход
		{
			ii = ii - 1;//if ii==0 jump
			volt_setup_delay += 10;
			if(volt_setup_delay > 500)
				return 0;
		}
		else
		{
			//calibration of the ADC voltage offset photodiode
			while(voltmeter_value[ii] == 0.0)
			{
				//insert voltmeter value in Watch1
			}
		}
		
		
		
	}//цикл по точкам калибровки
	

	//определение параметров по МНК
	//x - code_tec
	//y - adc conv result value
	//z - calibrated value (after calc)
	
	// y = C0 + C1 * x;
	uint8_t resp = 0;
	resp = calc_OLS( code_float, value_tec, num_points , &HV_coeff.C1, &HV_coeff.C0, value_eps);
	
	// z = B0 + B1 * x;
	resp += calc_OLS( code_float, voltmeter_value, num_points , &HV_coeff.B1, &HV_coeff.B0, value_eps);
	
	if(resp == 2)
	{
		HV_coeff.alpha1 =  HV_coeff.B1 / HV_coeff.C1 ;
		HV_coeff.alpha0 = HV_coeff.B0 - HV_coeff.C0 ;
		HV_coeff.beta0 = HV_coeff.C0 * ((double)1.0 - HV_coeff.alpha1) + HV_coeff.alpha0 ;
		//z = beta0 + alpha1 * y;
		
			
		HV.IsCalibrate = 1;
		HV.IsErrors = 0;
			
		//проверка вычислений
	
			//вычисляем код
		//value to code
		uint16_t code_calc = 0;
		//HV.code = HV.HV_min * (1.0/HV_coeff.B1) - HV_coeff.B0/ HV_coeff.B1 ; 
		code_calc = (uint16_t)(( HV.HV_target -  HV_coeff.B0 ) / HV_coeff.B1) ;
		
		HV.code = code_calc ;
			//вычисляем теор.значение
		// z = B0 + B1 * x;
		//code to value
		HV.HV_value = (float)(HV_coeff.B0 + HV_coeff.B1 * (double)HV.code) ;

			//настраиваем ЦАП
		DAC_EXT_write(HV.code);
		HAL_Delay(5000);
		
			//считываем АЦП
		conv_resp = 0;
		while(conv_resp == 0)
			conv_resp = SDADC_conv (SDADC_chn, &readed_data );
			//получаем сырое значение
		//get <y> value
		y0 = HV.HV_Kadc * floats[SDADC_chn] ;
		
			//получаем калиброванное значение
		// z = beta0 + alpha1 * y;
		//code to value
		y1 = (float)(HV_coeff.beta0 + HV_coeff.alpha1 * (double)y0) ;
		
			//смотрим разницу
		delta_y = HV.HV_value - y1 ;
	

		return 1;
	}
	else
		return 0;
}

//z = b0 + b1*x
//value = b0 + b1 * code
uint8_t HV_code_to_value (uint16_t inp_code, float *outp_volt)
{
	if (HV.IsCalibrate == 0)
		return 0;
	
	*outp_volt = ( inp_code * HV_coeff.B1 + HV_coeff.B0) ;
	if ( (*outp_volt >= HV.HV_min) && (*outp_volt <= HV.HV_max) )
	{
		HV.code = inp_code ;
		HV.HV_value = *outp_volt;
		return 1;
	}
	
	*outp_volt = 0.0 ;
	return 0;
}

//code = b0' + b1' * value
uint8_t HV_value_to_code (float inp_volt_val, uint16_t *outp_code)
{
	if (HV.IsCalibrate == 0)
		return 0;
	
	*outp_code = ( inp_volt_val  -  HV_coeff.B0) / HV_coeff.B1 ;
	if((*outp_code <= 0x3FBB)&&(*outp_code >= 0x0100))
	{
		HV.code = *outp_code ;
		HV.HV_value = inp_volt_val ;
		return 1;
	}
	return 0;
}

//коррекция измеренных АЦП данных с помощью проведенной калибровки
//z = beta0 + alpha1 * y
uint8_t HV_calib_res (float inp_adc_value, float *outp_value)
{
	if ( (HV.IsCalibrate == 0))//||(DET.IsPowerOn == 0) )
		return 0;
	
	*outp_value = ( inp_adc_value * HV_coeff.alpha1 + HV_coeff.beta0) ;
		return 1;
}

//установка напряжения по запрашиваемому значению
uint8_t HV_value_set_outp (float *inp_value)
{
	if((inp_value != NULL)&&(DET.IsPowerOn != 0))
	{
		if((*inp_value <= HV.HV_max) && (*inp_value >= HV.HV_min ))
		{
			int32_t code_sign = (int32_t)(( *inp_value  -  HV_coeff.B0) / HV_coeff.B1)  ;
			if (code_sign >=0 )
			{
				HV.code = (uint16_t)code_sign;
				HV.HV_value = (float)( HV.code * HV_coeff.B1 + HV_coeff.B0 );
				DAC_EXT_write(HV.code);
				return 1;
			}
			else
				return 0;
		}
		
	}
	else
		return 0;
	
	return 1;
}

//установка напряжения по запрашиваемому коду
uint8_t HV_code_set_outp (uint16_t *inp_code)
{
	if((inp_code != NULL)&&(DET.IsPowerOn != 0))
	{
		//uint16_t code_min = (uint16_t)(((double)HV.HV_min - HV_coeff.B0) / HV_coeff.B1);
		//uint16_t code_max = (uint16_t)(((double)HV.HV_max - HV_coeff.B0) / HV_coeff.B1);
		//if((*inp_code >= code_min) && (*inp_code <= code_max ))
		//{
			//float volt = (int32_t)( (double)(*inp_code) * HV_coeff.B1  +  HV_coeff.B0 ) ;
			//if( (volt <= HV.HV_max) && (volt >= HV.HV_min) )
			//{
				HV.code = *inp_code;
				//HV.HV_value = (float)( HV.code * HV_coeff.B1 + HV_coeff.B0 );
				DAC_EXT_write(HV.code);
				return 1;
			//}
			//else
			//	return 0;
		//}
		
	}
	else
		return 0;
	
	return 1;

}
