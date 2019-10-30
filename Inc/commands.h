#ifndef COMMANDS_H
#define COMMANDS_H


// Commands:

#define CMD_GetData 0 // Телеметрия
#define CMD_Setup_SPD_controller 1 // Настройка контроллера
#define CMD_DefaultSetPID 2 // Настройка параметров модели ПИД-регулятора
#define CMD_CalibThermometr 3 // Калибровка измерителя температуры 
#define CMD_SetDCDC 4 // Настройки TEC-драйвера
#define CMD_SetHV 5 // Настройки источника напряжения смещения
#define CMD_SPD_DDS 6 // Настройки синтезатора частот (DDS)
#define CMD_SerialNumber 7 // Установка/чтение серийного номера
#define CMD_SPD_NAME_WRITE 8 // Установка наименования
#define CMD_SPD_NAME_READ 9 // Чтение наименования
#define CMD_SPD_Status 10 // Чтение статуса детектора
#define CMD_Number_SPD 11 // номер ДОФ (запись)
#define CMD_Ahtung	12 // Аварийный режим работы

#pragma pack(push,1)
/*
    GetData: Телеметрия
*/

// To host:
typedef struct {
    float thrm_heatsink; // температура радиатора
    float thrm_VD; // температура фотодиода
    float TEC_voltage; // напряжение TEC-драйвера
    float Bias_VD; // смещение фотодиода
    float RF_PWR_SNS; // Датчик ВЧ мощности
    float thrm_used; // уставка температуры
    float hv_bias_used; // уставка смещения (расчет)
    unsigned short hv_code_used; // код смещения
    unsigned short stb_code_used; // код строба
    unsigned char Number_SPD; // номер детектора
    unsigned char Ready_SPD; // Готовность детектора
//		float Overheat_temp_status;	//Температура перегрева 
		float Dumping_temp_telemetry;	//сброс температуры до заданного значения 
		float Leaving_temp_telemetry;
		unsigned char EN_TIMER_OVERHEAT_telemetry; // флаг включение таймера сброса температуры
		unsigned char EN_TEC1_telemetry;	// флаг включения одностадийника при перегреве
		unsigned char EN_TEC3_telemetry;	//	флаг включения трехстадийника при перегреве
	
} GetData_TX_t;

/*
    Setup_SPD_controller: Настройка контроллера
*/

// To host:
typedef struct {
    float Tset; // Уставка TU1
    float HV_bias; // Напряжение смещения фотодиода
    unsigned short HV_code; // код смещения фотодиода
    unsigned short DDS_stb_code; // код амплитуды строба
		
} Setup_SPD_controller_TX_t;

// From host:
typedef struct {
    float Tset; // Уставка TU1
    float HV_bias; // Напряжение смещения фотодиода
    unsigned short HV_code; // код смещения фотодиода
    unsigned short DDS_stb_code; // код амплитуды строба
    unsigned short SPD_reset; // Запрос сброса питания детектора
    unsigned short MCU_RESET; // Запрос сброса контроллера
//		float Overheat_temp // Температура перегрева радиатора
} Setup_SPD_controller_RX_t;

/*
    DefaultSetPID: Настройка параметров модели ПИД-регулятора
*/

// To host:
typedef struct {
    float p_coeff; // Коэффициент P
    float i_coeff; // Коэффициент I
    float d_coeff; // Коэффициент D
    float T_set; // Уставка температуры
    float T_err; // Точность установки температуры
    unsigned long PWR_RCVR_DELAY; // Задержка включения питания детектора
    unsigned long adc_filtr; // длина фильтра АЦП
} DefaultSetPID_TX_t;

// From host:
typedef struct {
    unsigned short SPD_security_key; // Ключ разрешения записи настроек
    float p_coeff; // Коэффициент P
    float i_coeff; // Коэффициент I
    float d_coeff; // Коэффициент D
    unsigned long PWR_RCVR_DELAY; // Задержка включения питания детектора
    unsigned long adc_filtr; // длина фильтра АЦП
} DefaultSetPID_RX_t;

/*
    CalibThermometr: Калибровка измерителя температуры 
*/

// To host:
typedef struct {
    float Kr_ret; // Коэфф. tg(alfa) графика термодатчика
    float R0_ofs; // Коэфф. offset графика термодатчика
    float tg_tmpr_corr; // коррекция tg_tmpr_corr
    float ofs_tmpr_corr; // коррекция ofs_tmpr_corr
} CalibThermometr_TX_t;

// From host:
typedef struct {
    unsigned short SPD_security_key; // Ключ разрешения записи настроек
    float Kr_ret; // Коэфф. tg(alfa) графика термодатчика
    float R0_ofs; // Коэфф. offset графика термодатчика
    float tg_tmpr_corr; // коррекция tg_tmpr_corr
    float ofs_tmpr_corr; // коррекция ofs_tmpr_corr
} CalibThermometr_RX_t;

/*
    SetDCDC: Настройки TEC-драйвера
*/

// To host:
typedef struct {
    float dc_voltage_eps; // погрешность установки напряжения TEC
    float max_dc_volt; // максимальное напряжение TEC
    float max_dc_curr; // предельный ток TEC
    float tg_calc; // мультипл. коэфф. DC-регулятора
    float ofs_calc; // аддитив. коэфф. DC-регулятора
} SetDCDC_TX_t;

// From host:
typedef struct {
    unsigned short SPD_security_key; // Ключ разрешения записи настроек
    float dc_voltage_eps; // погрешность установки напряжения TEC
    float max_dc_volt; // максимальное напряжение TEC
    float max_dc_curr; // предельный ток TEC
    float tg_calc; // мультипл. коэфф. DC-регулятора
    float ofs_calc; // аддитив. коэфф. DC-регулятора
} SetDCDC_RX_t;

/*
    SetHV: Настройки источника напряжения смещения
*/

// To host:
typedef struct {
    float HV_bias_preset; // Предустановленное напряжение смещения фотодиода
    float HV_bias_min; // Минимальное напряжение смещения
    float HV_bias_max; // Максимальное напряжение смещения
    float HV_slope; // мультипликативная константа калибровки
    float HV_offset; // аддитивная константа калибровки
    unsigned short preset_code; // код предустановленного напряжения
} SetHV_TX_t;

// From host:
typedef struct {
    unsigned short SPD_security_key; // Ключ разрешения записи настроек
    float HV_slope; // мультипликативная константа калибровки
    float HV_offset; // аддитивная константа калибровки
} SetHV_RX_t;

/*
    SPD_DDS: Настройки синтезатора частот (DDS)
*/

// To host:
typedef struct {
    unsigned long long DDS_FTW; // Подстройка частоты
    unsigned short DDS_PHS; // Подстройка фазы
} SPD_DDS_TX_t;

// From host:
typedef struct {
    unsigned short SPD_security_key; // Ключ разрешения записи настроек
    unsigned short DDS_PHS; // Подстройка фазы
    unsigned long long DDS_FTW; // Подстройка частоты
} SPD_DDS_RX_t;

/*
    CMD_SerialNumber: Установка/чтение серийного номера
*/

// To host:
typedef struct {
    unsigned long serial_number_SPD; // Серийный номер
    unsigned long SPD_soft_version; // Версия ПО микроконтроллера
    unsigned short date_day; // день производства
    unsigned short date_month; // месяц производства
    unsigned short date_year; // год производства
} CMD_SerialNumber_TX_t;

// From host:
typedef struct {
    unsigned short SPD_security_key; // Ключ разрешения записи настроек
    unsigned long serial_number_SPD; // Серийный номер
    unsigned long SPD_soft_version; // Версия ПО микроконтроллера
    unsigned char date_day; // день производства
    unsigned char date_month; // месяц производства
    unsigned short date_year; // год производства
} CMD_SerialNumber_RX_t;

/*
    SPD_name_write: Установка наименования
*/

// To host:
typedef struct {
    unsigned long long SPD_name_txt; // Строка
} SPD_name_write_TX_t;

// From host:
typedef struct {
    unsigned short SPD_name_txt; // Строка текстового дескриптора
} SPD_name_write_RX_t;

/*
    SPD_name_read: Чтение наименования
*/

// To host:
typedef struct {
    unsigned short SPD_name_txt; // Строка
} SPD_name_read_TX_t;

/*
    SPD_status: Чтение статуса детектора
*/

// To host:
typedef struct {
    unsigned long SPD_status; // флаги
} SPD_status_TX_t;

/*
    Number_SPD: номер ДОФ (запись)
*/

// To host:
typedef struct {
    unsigned char SPD_num; // Номер детектор
} Number_SPD_TX_t;

// From host:
typedef struct {
    unsigned char SPD_num; // Строка текстового дескриптора
} Number_SPD_RX_t;

/*
    Ahtung: Аварийные режимы
*/

//To host:
typedef struct {
	float Dumping_temp;		//сброс температуры до заданного значения 
	//float Overheat_temp;	// перегрев радиатора
	float Leaving_temp;
	unsigned char EN_TIMER_OVERHEAT;// флаг включение таймера сброса температуры
//	unsigned char EN_TEC1_FLAG;		//флаг работы одностадийного элемента Пельтье при перегреве
//	unsigned char EN_TEC3_FLAG;		//флаг работы трехстадийного элемента Пельтье при перегреве
	
} Ahtung_TX_t;

//Fron host
typedef struct {
	float Dumping_temp;		//сброс температуры до заданного значения 
	//float Overheat_temp;	// перегрев радиатора
	float Leaving_temp;
	unsigned char EN_TIMER_OVERHEAT; // флаг включение таймера сброса температуры
//	unsigned char EN_TEC1_FLAG;		//флаг работы одностадийного элемента Пельтье при перегреве
//	unsigned char EN_TEC3_FLAG;		//флаг работы трехстадийного элемента Пельтье при перегреве
} Ahtung_RX_t;

#pragma pack(pop)

#endif // COMMANDS_H
