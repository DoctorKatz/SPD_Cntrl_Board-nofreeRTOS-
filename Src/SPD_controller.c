#include "SPD_controller.h"
#include "fifo_buffer.h"

//начальная инициализация параметров ПИД-прегулятора
void PID_init(PID_regs *, PID_state *);
void SPD_init(SPD_regs *);
void HV_source_init(HV_src *);
void DDS_init(DDS_regs *DDS);
void DET_init(DET_control *DET);
extern float abs_value(uint8_t *sign, float value);
//общие функции для использования различными модулями контроллера
uint8_t calc_OLS(float *inp_x, float *inp_y, uint16_t size, double *outp_a, double *outp_b, float *outp_eps);//Ordinary Least Squares

void PID_init(PID_regs *PID, PID_state *PID_st)
{
	
		//инициализирование констант
		PID->P_coeff = 0.5;
		PID->I_coeff = 0.01;
		PID->D_coeff = 10.0;
		PID->T_set = -50.0;//град. Цельсия
		PID->T_err = 0.5;//град. Цельсия
		PID->T_heatsink_max = 70.0;//град. Цельсия
		
		
		PID->adc_filtr = 96 ;//50SPS
		PID->smooth_filtr = 1;//усреднение выключено
		//формула из даташита АЦП для фильтра SINC4 с включенной(!!!) опцией Zero Latency
		PID->adc_sps = 614400.0/(128.0 * PID->adc_filtr * PID->smooth_filtr * ADC_SUB_CHANNELS);//[SPS]
		
		//PID->I_coeff = PID->adc_sps * PID->I_coeff;
		//PID->D_coeff = PID->D_coeff / PID->adc_sps;
		
		PID->factor_TEC_CURRENT = 5.990*5.0;
		PID->factor_TEC_VOLTAGE = (float)(1.0 + 10.0/3.3);//3.005;
	
		
		//параметры из спецификации задействованного элемента Пельтье
		PID->dc_voltage_eps = 0.1;//V
		PID->max_dc_volt = 6.0;//V
		PID->min_dc_volt = 0.6;//V
		PID->max_dc_curr = 6.5;//A

		//инициализируем переменные регулятора
		PID_st->state = 0;
		PID_st->mode = 1;//0-выкл, 1 - рабочий, 2 - пауза
		PID_st->trg_err_new = 0;
		PID_st->trg_err_old = 0;
		PID_st->prp_err = 0;
		PID_st->dif_err = 0;
		PID_st->int_err = 0;
		PID_st->error_new = 0;
		PID_st->error_old = 0;
		PID_st->response_mode = 0;//тип отклика пропорц.-интегр.-дифф.
		
		PID_st->timer_pwr_on = 90;//секунды
		

}//PID_init


void SPD_init(SPD_regs *SPD)
{
		SPD->Mode = 1;//нормальный режим работы
		SPD->state = 1;//ON
		SPD->T_heatsink = 25;
//		SPD->T_emerg = 45;//порог температуры при котором срабатывает предупреждение о перегреве
		SPD->secur_key = 1234;
		SPD->date_day = 01;
		SPD->date_month = 01;
		SPD->date_year = 2017;
		SPD->serial_number = 1;
		SPD->soft_version = 0x3;
		SPD->SPD_status_reg = 0;
}

void HV_source_init(HV_src *HV_regs)
{
	HV_regs->code = 0x5FFF ;
	HV_regs->HV_value = 14.8;
	HV_regs->HV_eps = 0.005 ;
	HV_regs->HV_max = 87.0 ;
	HV_regs->HV_min = 5.0 ;
	HV_regs->HV_target = 40.0 ;
	//HV_regs->HV_Kadc = (float)(2.5 / 4096.0);
	//HV_regs->HV_Kadc *= 31.9;//коэфф. деления резистивного делителя 1 МОм/32,4 кОм
	HV_regs->HV_Kadc = 37.18;//коэфф. деления резистивного делителя 1 МОм/(32,4||191) кОм

	HV_regs->IsErrors = 0;
	HV_regs->IsCalibrate = 0;
	HV_regs->state = 0;
}


void DDS_init(DDS_regs *DDS)
{
	DDS->IsErrors = 0;
	DDS->PHS = 0x0;
	DDS->FTW = 0x500000000000;
	DDS->STB = 0x0FF;
	DDS->pwrdwn = 0x0;
}

void DET_init(DET_control *DET)
{
	DET->IsCalibrate = 0;
	DET->IsErrors = 0;
	DET->IsPowerOn = 0;
}

uint8_t calc_OLS(float *inp_x, float *inp_y, uint16_t size, double *outp_a, double *outp_b, float *outp_eps)//Ordinary Least Squares
{
	if ((inp_x == NULL) || (inp_y == NULL) || (size < 4) )
		return 0;
	
	//float arr_x0 = *inp_x;//x0 - normalized
	double summ_x = 0.0;
	double summ_x2 = 0.0;
	double summ_xy = 0.0;
	double summ_y = 0.0;
	float *ptr_arr_x = inp_x;
	float *ptr_arr_y = inp_y ;
	float *ptr_arr_eps = outp_eps ;
	double temp = 0;

	for(uint32_t ii = 0; ii < size ; ii++)
	{
//		summ_x += (*ptr_arr_x - arr_x0) ;
//		summ_x2 += (*ptr_arr_x - arr_x0) * (*ptr_arr_x - arr_x0) ;
//		summ_y += *ptr_arr_y ;
//		summ_xy += (*ptr_arr_x - arr_x0) * (*ptr_arr_y) ;
		summ_x += (*ptr_arr_x) ;
		summ_x2 += (*ptr_arr_x) * (*ptr_arr_x) ;
		summ_y += *ptr_arr_y ;
		summ_xy += (*ptr_arr_x) * (*ptr_arr_y) ;
		ptr_arr_x++ ;
		ptr_arr_y++ ;
	}
	
	temp = (double)size * summ_x2 - summ_x * summ_x ;
	if(temp == 0)
		return 0;
	*outp_a = ((double)size * summ_xy - summ_x * summ_y) / temp;
	*outp_b =  (summ_y - (*outp_a) * summ_x) / (double)size;
	
	ptr_arr_x = inp_x;
	ptr_arr_y = inp_y ;
	for(uint16_t ii = 0; ii < size; ii++)
	{
		//temp = (*outp_a) * (*ptr_arr_x - arr_x0) + (*outp_b);//approx_line value
		temp = (*outp_a) * (*ptr_arr_x) + (*outp_b);//approx_line value
		uint8_t sign;
		float eps = (temp - (*ptr_arr_y));//difference with approx_line and real_value
		*ptr_arr_eps = abs_value(&sign, eps);
		ptr_arr_x++ ;
		ptr_arr_y++ ;
		ptr_arr_eps++ ;
	}
	
	return 1;
}

