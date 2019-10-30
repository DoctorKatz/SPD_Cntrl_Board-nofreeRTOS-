#include "DCDC_TEC.h"
#include "SPD_controller.h"

extern I2C_HandleTypeDef hi2c1;
extern void Error_Handler(void);
extern uint8_t select_chn_ADC(uint8_t channel);
extern ADC_HandleTypeDef hadc1;
extern PID_regs PID;
extern PID_state PID_st;
extern void MX_I2C1_Init(void);
extern float abs_value(uint8_t *sign, float value);

extern float T_target ;
extern float T_lim ;
extern uint8_t tec_pwr_lim ;
extern float prop_coeff;

//непосредственно таблица с данными
uint16_t code_tec[table_size];//управляющий код
uint8_t valid_tec[table_size];//флаг валидности
float value_tec[table_size];//значение напряжения в единицах СИ
float value_eps[table_size];//значение погрешности между аппрокс. и реальными отсчетами
float code_float[ table_size ];//вещественная ось абсцисс

float max_volt;
float min_volt;
uint8_t ind_max_volt;
uint8_t ind_min_volt;
//float full_scale_volt;
	
uint16_t *ptr_code;
uint8_t *ptr_valid;
float *ptr_value;

uint16_t *ptr_code_tmp;
uint8_t *ptr_valid_tmp;
float *ptr_value_tmp;

//указатели на столбцы таблицы
//calib_table_addr ptr_table;
//uint8_t *ptr_code_tec = code_tec;
//uint8_t *ptr_valid = valid;
//float *ptr_phys_tec = phys_value_tec;

//опорные точки таблицы калибровки
//calib_table_param tab_tec;
//float max_tec_volt, min_tec_volt;
//uint8_t ind_max_tec_volt, ind_min_tec_volt;

void ptr_calib_table_init(void);
void VarRes_reset(void);
uint8_t TEC_DRIVER_INIT(void);
void TEC_DRIVER_TEST(void);
uint8_t VarRes_write(uint8_t DATA);
uint8_t VarRes_read(uint8_t *DATA);
uint8_t calibrate_TEC_DRIVER(void);
//uint8_t max_phys_value_TEC(void);
//uint8_t min_phys_value_TEC(void);
uint8_t value2code_TEC(float target_volt);//функция методом перебора значений - используется при вычислении коэффициентов линейной аппроксимации
int calc_value2code_TEC(float target_volt);//функция методом вычисления - используется в рабочем режиме

uint8_t tec_volt_cntr(float percent);//percent = 0..1
float calc_response_PID(float input, float input_previous);//input - температура, previous - значение предыдущего отсчета


void ptr_calib_table_init(void)
{
	 ptr_code = code_tec;
	 ptr_valid = valid_tec;
	 ptr_value = value_tec;
	
	 ind_max_volt = 0;
	 ind_min_volt = 0;
	 max_volt = 0;
	 min_volt = 0;
}


void VarRes_reset(void)
{
	//сброс потенциометра
	HAL_GPIO_WritePin(GPIOB, I2C1_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(4);
	HAL_GPIO_WritePin(GPIOB, I2C1_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

uint8_t TEC_DRIVER_INIT(void)
{
	uint8_t VarRes_write_code = 0xF0;
	uint8_t VarRes_read_code = 0;
	VarRes_reset();
	
	VarRes_write(VarRes_write_code);
	VarRes_read(&VarRes_read_code);
	if(VarRes_read_code == VarRes_write_code)
	{
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
		return 1;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
		//Error_Handler();
		return 0;
	}
}


uint8_t VarRes_write(uint8_t DATA)
{
		HAL_GPIO_WritePin(GPIOD, TP_I2C1_Pin, GPIO_PIN_SET);
		uint8_t addr = 0; //аппаратный адрес устройства
		uint8_t command = 0; //байт команды
		uint8_t tx24[2];
		HAL_StatusTypeDef status = HAL_OK;

		addr = 0x58;
		command = 0x00;
			
		tx24[0] = command;
		tx24[1] = DATA;
		//status = HAL_I2C_Master_Transmit(&hi2c1, addr, &tx24[0], 2 * sizeof(uint8_t), 0x1000);
		status = HAL_I2C_Master_Transmit(&hi2c1, addr, &tx24[0], 2 * sizeof(uint8_t), 0x100);
		HAL_GPIO_WritePin(GPIOD, TP_I2C1_Pin, GPIO_PIN_RESET);
		if (status==HAL_OK)
		return 1;
		else
		return 0;
}

uint8_t VarRes_read(uint8_t* DATA)
{
		HAL_GPIO_WritePin(GPIOD, TP_I2C1_Pin, GPIO_PIN_SET);	
		uint8_t addr = 0; //аппаратный адрес устройства
		HAL_StatusTypeDef status = HAL_OK;
	
		addr = 0x58;
		addr |= 0x01;
		status = HAL_I2C_Master_Receive(&hi2c1, addr, DATA, sizeof(uint8_t), 0x1000);
		HAL_GPIO_WritePin(GPIOD, TP_I2C1_Pin, GPIO_PIN_RESET);
		if (status==HAL_OK)
		return 1;
		else
		return 0;
}


void TEC_DRIVER_TEST(void)
{
//после инициализации значения сопротивления потенциометра производится
	//включение TEC драйвера 
	uint8_t code;
	HAL_GPIO_WritePin(GPIOD, PWR_TEC_CNTR_Pin, GPIO_PIN_RESET);
	
	while(1)
	{
		for(uint8_t i = 0; i < table_size ; i++)
		{
			HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
			code = i;
			VarRes_write(code);
			HAL_Delay(100);
			HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
			HAL_Delay(100);
		}
	}
}


uint8_t calibrate_TEC_DRIVER(void)
{
		uint8_t ret_val = 1;
		uint8_t	VarRes_value = 0;
		uint8_t ADC_chn_sel = 0;
		uint16_t adc_res_uint = 0;
		float	adc_res_float = 0.0;
		float TB3_current = 0.0;
		float TB3_voltage = 0.0;
		//GPIO_PinState PWR_GOOD = GPIO_PIN_RESET;
	
	//инициализация значений указателей на таблицу калибровки
		ptr_calib_table_init();
	
		HAL_GPIO_WritePin(GPIOD, PWR_TEC_CNTR_Pin, GPIO_PIN_SET);//TEC TB3 OFF
		HAL_Delay(200);
		VarRes_value = 0;
		VarRes_write(VarRes_value);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOD, PWR_TEC_CNTR_Pin, GPIO_PIN_RESET);//TEC TB3 ON
		HAL_Delay(2000);

		//проверяем напряжение при свипировании входного кода -
		//заполняем таблицу калибровки DC-DC-преобразователя
		for(uint16_t i = 0; i < table_size; i++)
		{
				VarRes_value = i;
				VarRes_write(VarRes_value);
				HAL_Delay(10);
				VarRes_value = 0;
				VarRes_read(&VarRes_value);
				HAL_Delay(300);
				
				valid_tec[i] = 0;	
//				PWR_GOOD = GPIO_PIN_RESET;
//				PWR_GOOD = HAL_GPIO_ReadPin(GPIOD, PG_TEC_Pin);
//				if( (i==(uint16_t)VarRes_value) && (PWR_GOOD==GPIO_PIN_SET) )
				if( i == (uint16_t)VarRes_value )
				{
							valid_tec[i] = 1;
						//измеряем ток через TB3
							ADC_chn_sel = 0;
							select_chn_ADC(ADC_chn_sel);
							HAL_ADC_Start(&hadc1);
							hadc1.Instance->CR2 |= 0x40000000;
							TB3_current = 0;
							if(HAL_ADC_PollForConversion(&hadc1, 10000) == HAL_OK)
							{
								adc_res_uint = HAL_ADC_GetValue(&hadc1);
								HAL_ADC_Stop(&hadc1);
								adc_res_uint &= 0x0FFF;
								adc_res_float = (adc_res_uint * (float)(2.5/4096));
								//TB3_current = adc_res_float * (float)(5.990*5.0);	
								TB3_current = adc_res_float * PID.factor_TEC_CURRENT ;
							}

						//измеряем напряжение на выходе драйвера TB3					
							float mean_value_volt = 0.0;
							uint32_t num_points = 8;
							for(uint16_t cntr = 0; cntr < num_points; cntr++)
							{
								ADC_chn_sel = 1;
								select_chn_ADC(ADC_chn_sel);
								HAL_ADC_Start(&hadc1);
								hadc1.Instance->CR2 |= 0x40000000;
								TB3_voltage = 0;
								if(HAL_ADC_PollForConversion(&hadc1, 10000) == HAL_OK)
								{
									adc_res_uint = HAL_ADC_GetValue(&hadc1);
									HAL_ADC_Stop(&hadc1);
									adc_res_uint &= 0x0FFF;
									adc_res_float = (adc_res_uint * (float)(2.5/4096));						
									//mean_value_volt += adc_res_float * (float)3.005;	
									mean_value_volt += adc_res_float * PID.factor_TEC_VOLTAGE;
								}
							}
							TB3_voltage = mean_value_volt / num_points;	
							//HAL_ADC_Stop(&hadc1);
					//если ток через ТВ3 превышает лимит, то сбросить значение флага
							code_tec[i] = (uint8_t)i;
							if (TB3_current <= PID.max_dc_curr)
							{

									value_tec[i] = TB3_voltage;
							}
							else
							{

								value_tec[i] = 0;
							}

							
				}
				else
				{

					code_tec[i] = (uint8_t)i;

					value_tec[i] = 0;
				}
			
			}//for по таблице калибровке
		
			//для построения уравнения линейной аппроксимации необходимо найти положение опорных точек:
			//поиск максмимального и минимального значений
			
			//max_phys_value_TEC();
			float temp;
			uint8_t ind;
			
			temp = value_tec[0];
			ind = code_tec[0];
			for (uint16_t i = 0; i < table_size ; i++)
			{
				if( ( value_tec[i] >= temp )&&( value_tec[i] != (float)0.0 ) )
				{
					temp = value_tec[i];
					ind = code_tec[i];
				}
			}	
			ind_max_volt = ind;
			max_volt = temp;
			
			//min_phys_value_TEC();
			temp = max_volt;
			ind = ind_max_volt;
			for (uint16_t i = 0;  i < table_size; i++)
			{
				if( ( value_tec[i] < temp )&&( value_tec[i] != (float)0.0 ) )
				{
					temp = value_tec[i];
					ind = code_tec[i];
				}
			}	
			ind_min_volt = ind;
			min_volt = temp;
			
			//переносим данные значения в структуру
			if ( max_volt > PID.max_dc_volt)
			{
				//переопределяем индекс максимального значения
				 ind_max_volt = value2code_TEC(PID.max_dc_volt);
				 max_volt = PID.max_dc_volt;//ограничиваем максимальное напряжение на ТВ3
			}
			else
			{
				PID.max_dc_volt =  max_volt;//корректируем ранее установленное значение по умолчанию
			}
		
			PID.min_dc_volt =  min_volt;
			
			if ( ind_max_volt !=  ind_min_volt )
			{
				uint8_t sign = 0;
				PID.dc_voltage_eps = ( max_volt -  min_volt) / abs_value(&sign, (float)(ind_min_volt -  ind_max_volt)) ;
			}
			else
				return 0;//ошибка деления на ноль
			
			//вычисление коэффициентов линейной аппроксимации
			if(( ind_max_volt!= ind_min_volt) && ( max_volt !=  min_volt) )
			{
				//заменить дробь на метод наименьших квадратов
				PID_st.tg_calc = ( max_volt -  min_volt) /  (int)( (uint8_t)ind_max_volt + (-1)*(uint8_t)ind_min_volt);
				PID_st.ofs_calc =  max_volt - PID_st.tg_calc *  (int)ind_max_volt;
				PID_st.tg_calc = (float)1.0 / PID_st.tg_calc;
				//full_scale_volt = (float)1.0/max_volt;
				//full_scale_volt = 1.0/(max_volt -  min_volt);
				PID.index_max_volt = (uint32_t)ind_max_volt;
				PID.index_min_volt = (uint32_t)ind_min_volt;
				ret_val = 1;
			}
			else
			{
				PID_st.tg_calc = (float)0.01;
				PID_st.ofs_calc = (float)0.0;
				ret_val = 0;
			}
		
			
			
		VarRes_value = calc_value2code_TEC(PID.min_dc_volt);
		VarRes_write(VarRes_value);
		HAL_Delay(10);
		//инициализация DC-DC END****************************************************
		return ret_val;
}


uint8_t value2code_TEC(float target)
{
	float voltage_eps = ( max_volt -  min_volt)/(float)( ind_max_volt -  ind_min_volt);
	if (voltage_eps < 0) voltage_eps *= -1.0;
	float level_up = target + voltage_eps/(float)2.0;
	float level_dn = target - voltage_eps/(float)2.0;
	float voltage;
	uint8_t ret_code;

//	ptr_code_tmp =  ptr_code;
//	ptr_value_tmp =  ptr_value;

	//проверка на соответствие диапазону выходных напряжений источника
	if (target >  max_volt)	return  ind_max_volt;
	if (target <  min_volt)	return  ind_min_volt;
	
	ret_code = 0;
	for(uint32_t i = 0; i < table_size; i++)
	{
		//voltage = *ptr_value_tmp;
		voltage = value_tec[i];
		if ( (voltage >= level_dn) && (voltage <= level_up) )
		{
			//ret_code = *ptr_code_tmp;
			ret_code = code_tec[i];
			break;
		}
//		ptr_code_tmp++;
//		ptr_value_tmp++;
	}
	
	return ret_code;
}

inline int calc_value2code_TEC(float target_volt)
{
	return (int)( (target_volt - PID_st.ofs_calc) * PID_st.tg_calc  );
}


uint8_t tec_volt_cntr(float percent)//percent = 0...1
{
	if (percent > tec_pwr_lim )
		percent = tec_pwr_lim;
	
	//float voltage = percent * PID.max_dc_volt;
	float voltage = (percent / 100) * (PID.max_dc_volt - PID.min_dc_volt) + PID.min_dc_volt;
	uint8_t var_res_code = 0;
	
	if (voltage >= PID.max_dc_volt)
	{
		voltage = PID.max_dc_volt;
		var_res_code = PID.index_max_volt;
	}
	
	if (voltage <= PID.min_dc_volt)
	{
		voltage = PID.min_dc_volt;
		var_res_code = PID.index_min_volt;
	}
	
	var_res_code = calc_value2code_TEC(voltage);
	
	if ( VarRes_write(var_res_code) )  
		return 1;
	else
	{
		HAL_GPIO_WritePin(GPIOB, I2C1_RST_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOB, I2C1_RST_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		if ( VarRes_write(var_res_code) )  
			return 1;
		else
		{
			Error_Handler();
		}

		return 0;
		
	}
}


float calc_response_PID(float input, float input_previous)//input - температура
{
	float resp = 0.0;
		
	PID_st.error_new = input - T_target;//значение ошибки в текущий момент времени
	PID_st.error_old = input_previous - T_target;// значение ошибки в пердыдущий момент времени
	//PID_st.error_new = input - PID.T_set;//значение ошибки в текущий момент времени
	//PID_st.error_old = input_previous - PID.T_set;// значение ошибки в пердыдущий момент времени
		
//	if (PID_st.error_new < 1.0) PID_st.response_mode = 1;
//	if (PID_st.error_new > 1.0) PID_st.response_mode = 2;
	
	
	PID_st.prp_err = PID_st.error_new * PID.P_coeff;
	//PID_st.prp_err = PID_st.error_new * prop_coeff;
	if (PID_st.response_mode == 2)//P
	{
		PID_st.int_err = 0;
		PID_st.dif_err = 0;
		PID_st.error_old = 0;
		return PID_st.prp_err;
	}
	resp = PID_st.prp_err;
	PID_st.int_err = PID_st.int_err + PID_st.error_new * PID.I_coeff;//  * delta_T
	resp += PID_st.int_err;
	
	//проверка на переполнение
	if( (PID_st.int_err >= (float)1.0) || (PID_st.int_err <= (float)(-1.0)) )
	{
		if (PID_st.int_err < 0)
			PID_st.int_err = -(float)1.0;
		else
			PID_st.int_err = (float)1.0;
	}
	if (PID_st.response_mode == 1)//PI
	{
		PID_st.dif_err = 0;
		PID_st.error_old = 0;
		return resp;
	}

	//PID_st.dif_err = (input - input_previous) * PID.D_coeff;
	PID_st.dif_err = (PID_st.error_new - PID_st.error_old) * PID.D_coeff;
	//PID_st.error_old = PID_st.error_new;
	resp += PID_st.dif_err;
	if (PID_st.response_mode == 0)//PID
	{
		return resp;
	}
	
	
	return 0.0;
}

////uint8_t max_phys_value_TEC(void)
////{
////	volatile float temp;
////	volatile uint8_t ind;
////	uint8_t i;
////	
////	ptr_code_tmp =  ptr_code;
////	ptr_value_tmp =  ptr_value;
////	
////	
////	temp = *ptr_value_tmp;
////	ind = *ptr_code_tmp;
////	for (i = 0; i < table_size ; i++)
////	{
////		if ( *ptr_value_tmp >= temp )
////		{
////			temp = *ptr_value_tmp;
////			ind = *ptr_code_tmp;
////		}
////		ptr_code_tmp++;
////		ptr_value_tmp++;
////	}	
////	ind_max_volt = ind;
////	max_volt = temp;
////	
////	return 1;
////	
////}

////uint8_t min_phys_value_TEC(void)
////{
////	volatile float temp;
////	volatile 	uint8_t ind;
////	uint8_t i;
////	
////	ptr_code_tmp =  ptr_code;
////	ptr_value_tmp =  ptr_value;

////	
////	temp = *ptr_value_tmp;
////	ind = *ptr_code_tmp;
////	for (i = 0;  i < table_size; i++)
////	{
////		if ( *ptr_value_tmp < temp )
////		{
////			temp = *ptr_value_tmp;
////			ind = *ptr_code_tmp;
////		}
////		ptr_code_tmp++;
////		ptr_value_tmp++;
////	}	
////	 ind_min_volt = ind;
////	 min_volt = temp;
////	
////	return 1;
////}


