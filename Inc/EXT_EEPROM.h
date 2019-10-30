#include "stm32f4xx_hal.h"

//адреса хранения данных в EEPROM
#define eeprom_buffer_size 0x00FF //размер буфера данных для обмена с EEPROM 64 элемента по 4 байта = 256 байт
#define error_CRC_eeprom (1 << 0)//флаг ошибки при проверке CRC
//#define TXT_DEV_DSC 0x0

#define BASE_DEV_DESC 0x0000 //дескриптор устройства 64 байта для хранения общих информации, серийного номера и настроек
#define OFS_TAB_PID_VALID 0x0001 //байт актуальности настроек регулятора 0
#define OFS_TAB_HV_VALID 0x0002 //байт актуальности настроек HV источника напряжения
#define OFS_TAB_PT_VALID 0x0003 //байт актуальности калибровки АЦП

#define BASE_HV_SET 0x0005 //адрес первого байта с последним установленым HV

#define BASE_T_SET 0x0010 //адрес первого байта с последним установленым Tset

#define BASE_STB_SET 0x0016 //адрес первого байта с последним установленым STB set

#define BASE_PID_SET 0x0020 //с этого адреса записываются настройки канала ПИД-регулятора
//настройки ПИД - это 21[регистр] х 4[байта] = 84 [байта] + 4 [байта CRC]

#define BASE_PT_SET ( BASE_PID_SET +  256 * 4 ) //данные калибровки АЦП
//набор коэфф.калибровки СД-АЦП - 6 регистров х 4 байта + 4 байта CRC

#define BASE_HV_DAC_SET ( BASE_PT_SET +  10 * 4 ) //сохранение коэффициентов калибровки HV-источника

#define STR_SPD_DESC (BASE_HV_DAC_SET + 10 * 8 )//строковый дескриптор - 62 байта + 4 байта

#define BASE_SPD_SERIAL (STR_SPD_DESC + 70 * 4 )//серийный номер

#define BASE_SPD_NUMBER (BASE_SPD_SERIAL + sizeof(SPD_regs) + 10*8) // номер детектор + 4 байта CRC
	
#define BASE_DUMPING_TEMP (BASE_SPD_NUMBER + sizeof(Detector_number_wr) + 10*8)//температура перегрева + 4 байта CRC
	
#define BASE_EN_TIMER_OVERHEAT_FLAG (BASE_DUMPING_TEMP + sizeof(overheat_achtung.dumping_temp) + 10*8)
	
#define BASE_LEAVING_TEMP (BASE_EN_TIMER_OVERHEAT_FLAG + sizeof(overheat_achtung.EN_TIMER_OVERHEAT_FLAG) + 10*8)



