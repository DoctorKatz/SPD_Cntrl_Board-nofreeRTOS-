#include "stm32f4xx_hal.h"
//#include "math.h"

//флаги состояний и ошибок SPD

////флаги состояния (1 и 2 байты)
#define SPD_ready_flag 					(1 << 0)//ДОФ готов ( к работе  )
#define SPD_err_flag 						(1 << 1)//наличие ошибок в работе ДОФ
#define SPD_warm_up_flag 				(1 << 2)//происходит выход на режим фотодетектора
#define SPD_DET_pwr_flag				(1 << 3)//питание детектора
//предупреждения
#define SPD_SDADC_calib_flag 		(1 << 4)//откалиброван АЦП
#define SPD_TEC_drv_calib_flag	(1 << 5)//откалиброван TEC драйвер
#define SPD_HV_drv_calib_flag		(1 << 6)//откалиброван HV


///флаги ошибок (3 и 4 байты)
//#define SPD_SHRED_err_flag 				(1 << 16)//несовпадение аппар.ключа и вресии прошивки
#define SPD_PT_err_flag 					(1 << 17)//не подключен термодатчик
//#define SPD_heatsink_sense_flag 	(1 << 18)//не подключен термодатчик радиатора
//#define SPD_TB3_disconnected_flag	(1 << 19)//не подключен Пельтье элемент
#define SPD_TEC_driver_err_flag 	(1 << 20)//ошибка TEC драйвера
#define SPD_SDADC_err_flag 				(1 << 21)//ошибка внешнего АЦП
#define SPD_EEPROM_err_flag 			(1 << 22)//ошибка обращения к памяти EEPROM

#define SPD_DDS_err_flag 					(1 << 23)//ошибка при обращении к DDS
#define SPD_HV_err_flag 					(1 << 24)//ошибка при обращении к HV

#define SPD_overheat_flag 				(1 << 25)//перегрев радиатора ДОФ

//#define SPD_calc_err_flag 				(1 << 26)//ошибка при вычислениях

#define SPD_emerg_stop_flag 			(1 << 31)//аварийная прекращение работы ДОФ









#define table_size 0xFF //калибровочная таблица DC-преобразователя
//enum TEC_chn
//{
//	TEC_APD_INT = 0,
//	TEC_APD_EXT,
//	TEC_RECEIVER,
//};

//структура, содержащая калибровочные коэффициенты и дополнительные настройки регулятора
typedef struct
{
	//параметры ПИД-регулятора
	float P_coeff;
	float I_coeff;
	float D_coeff;
	float T_set;//уставка
	float T_err;//границы диапазона, в котором температурный режим считается установившимся
	float T_heatsink_max;//верхний порог температуры основания радиатора...
	//...при которой происходит аварийное выключение ПИД-регулятора

	//параметры измерительного тракта температуры

	//параметры измерительного тракта напряжения и тока
	float factor_TEC_CURRENT;
	float factor_TEC_VOLTAGE;
	
	//параметры DC-DC-регулятора
	float dc_voltage_eps;//точность установки
	float	max_dc_volt;//верхняя граница управляющей характеристики и предельное значение напряжения
	float	min_dc_volt;//нижняя граница управляющей характеристики
	float	max_dc_curr;//предельный ток ТЕС элемента

		
	float adc_sps;//вычисленное значение следования отсчетов АЦП
	uint32_t index_max_volt;//индекс элемента max_volt в массиве
	uint32_t index_min_volt;//индекс элемента min_volt в массиве
	
	uint32_t adc_filtr;//значение внутреннего фильтра АЦП
	uint32_t smooth_filtr;//длина буфера сглаживающего фильтра, кратно степени числа 2

} PID_regs;

//структура, содержащая динамические переменные ПИД-регулятора
typedef struct
{
	float trg_err_new;//текущее рассогласование
	float trg_err_old;//текущее рассогласование
	float prp_err;//пропорциональная ошибка
	float int_err;//интегральная ошибка
	float dif_err;//дифференциальная ошибка (хранит предыдущий отсчет)
	float error_new;
	float error_old;
	
	uint32_t timer_pwr_on;//задержка включения сигнальной платы

	uint8_t response_mode;//0 - PID, 1 - PI, 2 - P
	////	//значения коэффициентов линейной аппроксимации...
////	//...для управляющей характеристики DC-DC-регулятора
	float tg_calc;
	float ofs_calc;
	uint32_t	mode;//задание режима работы регулятора: нормальный-2/пауза-1/выкл-0
	//запрос текущего состояния ПИД регулятора: наличие калиброван/ наличие ошибки/ текущий режим
	uint32_t state;
	unsigned IsPowerOn:1;//включение сигнльной платы
	unsigned IsCalibrate:1;
	unsigned IsErrors:1;
	
	//uint32_t CRC32_read;//CRC код считанных из EEPROM настроек
	//uint32_t CRC32_write;//CRC код записываемых в EEPROM настроек
} PID_state;


typedef struct
{
	uint32_t state;					//отображает состояние контроллера
	uint32_t T_heatsink;		//отображает темепратуру основания радиатора
	float T_emerg;				//максимальное значение аварийного отключения контроллера
	uint32_t Mode;					//задание предопределённого режима работы контроллера
	uint32_t serial_number;	//серийный номер
	uint32_t soft_version;	//версия микропрограммного обеспечения
	uint16_t secur_key;			//ключ разрешения записи настроек (модификации)
	uint16_t date_day;			//день производства
	uint16_t date_month;		//месяц производства
	uint16_t date_year;			//год производства
	
	uint32_t SPD_status_reg;//статус SPD в целом: 
	
} SPD_regs;


typedef struct
{
	float HV_max;
	float HV_min;
	float HV_target;
	float HV_eps;
	float HV_Kadc;
	float HV_value;
//	float tg_calib;
//	float ofs_calib;
//	float tg_back;
//	float ofs_back;
	uint16_t code;

	//unsigned IsPowerOn:1;
	unsigned IsErrors:1;
	unsigned IsCalibrate:1;
	uint8_t state;
} HV_src;



typedef struct
{
	unsigned long long	FTW;
	unsigned short	PHS;
	unsigned short	STB;
	unsigned IsErrors:1;
	unsigned short	pwrdwn;
}	DDS_regs;


//биты и флаги управления детектором
typedef struct
{
	unsigned IsPowerOn:1;
	unsigned IsErrors:1;
	unsigned IsCalibrate:1;
} DET_control;


