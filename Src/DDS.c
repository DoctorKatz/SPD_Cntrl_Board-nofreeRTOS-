#include "main.h"
#include "stm32f4xx_hal.h"
#include "SPD_controller.h"

extern DET_control DET;
extern SPD_regs SPD;
extern DDS_regs DDS;
extern void tim3_delay(void);

uint8_t write_DDS(uint16_t addr, uint8_t *data, uint8_t num_bytes);
uint8_t read_DDS(uint16_t addr, uint8_t *data, uint8_t num_bytes);
uint8_t init_DDS(void);
void sleep_DDS(void);
void wakeup_DDS(void);
uint8_t DDS_stb_write(uint16_t inp_code);
uint8_t DDS_stb_read(uint16_t *outp_code);

uint8_t write_DDS(uint16_t addr, uint8_t *data, uint8_t num_bytes)
{
	if(	DET.IsPowerOn == 0 )//&& (DDS.pwrdwn == 0))
		return 0;
	
	if((num_bytes == 0) || (data == NULL) || (addr > 0x0509) )
		return 0;
	
	uint8_t *ptr_data = data;
	uint8_t byte_cntr = 0;
	uint8_t buff = 0;
	uint16_t cmd = 0;
	uint16_t temp = 0;


		//W1,W0
	if (num_bytes < 4)
		byte_cntr = num_bytes - 1;
	else
		byte_cntr = 0x3;//stream mode
	
	HAL_GPIO_TogglePin(TP_SPI4_GPIO_Port, TP_SPI4_Pin);
	HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_RESET);//AD9912
	tim3_delay();
	
	//forming instruction and send first itself
	cmd |= (addr & 0x1FFF);//install addres
	cmd |= (byte_cntr & 0x3) << 13 ;//W1:W0
	cmd &= 0x7FFF ;//write operation flag
	for(uint8_t j = 0; j < 16; j++)//package 2 byte, data transfer starting from older byte
	{
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin , GPIO_PIN_RESET);
		temp =  cmd &( 1 << (15-j) );
		if (temp == 0)
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_SET);
		
		//writting data registr from front SCK
		tim3_delay();
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_SET);
		tim3_delay();
	}
	
	//forming the data following the command
	
	for(uint8_t jj = 0; jj < num_bytes ; jj++)
	{
		buff = *(ptr_data + jj) ;
		for(uint8_t k = 0; k < 8; k++)
		{
			HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
			temp = (uint16_t)(buff & (1 << (7 - k)));
			if (temp == 0)
				HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_SET);
			
			tim3_delay();
			HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_SET);
			tim3_delay();
		}
	}
	
	tim3_delay();
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET);//AD9912
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_TogglePin(TP_SPI4_GPIO_Port, TP_SPI4_Pin);
	
	return 1;
}

uint8_t read_DDS(uint16_t addr, uint8_t *data, uint8_t num_bytes)
{
	if((DET.IsPowerOn == 0) )//&& (DDS.pwrdwn == 0))
		return 0;
	
	if((num_bytes == 0) || (data == NULL) || (addr > 0x0509) )
		return 0;
	
	uint8_t *ptr_data = data;
	uint8_t byte_cntr = 0;
	uint8_t buff = 0;
	uint16_t cmd = 0;
	uint16_t temp = 0;
	
	//clean buffer reciving data
	for(uint16_t ii = 0; ii < num_bytes; ii++)
	 *(ptr_data + ii) = 0;
	
	//W1,W0
	if (num_bytes < 4)
		byte_cntr = num_bytes - 1;
	else
		byte_cntr = 0x3;//stream mode
	
	HAL_GPIO_TogglePin(TP_SPI4_GPIO_Port, TP_SPI4_Pin);
	HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_RESET);//AD9912
	tim3_delay();
	
	//forming instruction, which send first
	cmd |= (addr & 0x1FFF);//set up addres
	cmd |= (byte_cntr & 0x3) << 13 ;//W1:W0
	cmd |= 0x8000 ;//read operation flag
	for(uint8_t j = 0; j < 16; j++)//package 2 byte, data transfer starting from older byte
	{
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin , GPIO_PIN_RESET);
		temp = cmd & ( 1 << (15-j) );
		if (temp == 0)
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_SET);
		
		//writting data registr from front SCK SCK
		tim3_delay();
		HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_SET);
		tim3_delay();
	}
	
	//read the data following the command
	for(uint8_t jj = 0; jj < num_bytes ; jj++)
	{
		buff = 0;
		for(uint8_t k = 0; k < 8; k++)
		{
			HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_RESET);
			tim3_delay();
			if( HAL_GPIO_ReadPin(SPI4_MISO_M_GPIO_Port, SPI4_MISO_M_Pin) == GPIO_PIN_SET )
				buff |= (1 << (7 - k));
			else
				buff &= ~(1 << (7 - k));
			
			HAL_GPIO_WritePin(SPI4_SCK_M_GPIO_Port, SPI4_SCK_M_Pin, GPIO_PIN_SET);
			tim3_delay();
		}
		*(ptr_data + jj) = (uint8_t)buff ;
	}
	
	tim3_delay();
	HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);//MAX1932
	HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);//DAC8411
	HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_SET);//AD9912
	HAL_GPIO_WritePin(SPI4_MOSI_M_GPIO_Port, SPI4_MOSI_M_Pin, GPIO_PIN_RESET);
	HAL_GPIO_TogglePin(TP_SPI4_GPIO_Port, TP_SPI4_Pin);
	
	return 1;
}

uint8_t init_DDS(void)
{
	if(DET.IsPowerOn == 0)
	{
		HAL_GPIO_WritePin(PWR_DET_CNTR_GPIO_Port, PWR_DET_CNTR_Pin, GPIO_PIN_SET);//detector power ON
		DET.IsPowerOn = 1;
		HAL_Delay(500);
	}
			
	HAL_GPIO_WritePin(SPI4_PWDN_GPIO_Port, SPI4_PWDN_Pin, GPIO_PIN_RESET);
	tim3_delay();
	DDS.pwrdwn = 0;
	HAL_GPIO_WritePin(SPI4_RST_GPIO_Port, SPI4_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	//tim3_delay();
	HAL_GPIO_WritePin(SPI4_RST_GPIO_Port, SPI4_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	//tim3_delay();
	
	uint8_t resp = 0;
	uint16_t DDS_addr = 0x0 ;
	uint8_t DDS_buff[4] = {0x0, 0x0, 0x0, 0x0};
	uint8_t DDS_data_size = 1;
	DDS_buff[0] = 0x81;
	//SDO_active = 1 <=> setup 4-wire bus
	write_DDS(DDS_addr, DDS_buff, DDS_data_size);
	//HAL_Delay(2);
	//tim3_delay();
	
	
	for(uint8_t ii = 0; ii < 4; ii++)
			DDS_buff[ii] = 0x0 ;
	
	//request ID of DDS
	DDS_addr = 0x3;
	DDS_data_size = 2;
	read_DDS(DDS_addr, DDS_buff, DDS_data_size);
	//ID query 0x19 0x82
	if( (DDS_buff[0] == 0x19) && ( (DDS_buff[1]&0x3) == 0x02) )
	{
		//N-divider
		DDS_addr = 0x20;
		DDS_data_size = 1;
		DDS_buff[0] = 0x03;
		write_DDS(DDS_addr, DDS_buff, DDS_data_size);
		//tim3_delay();
		
		//FTW0 [47:40]
		DDS_addr = 0x01AB;
		DDS_data_size = 1;
		DDS_buff[0] = 0x50;
		write_DDS(DDS_addr, DDS_buff, DDS_data_size);
		//tim3_delay();
		
//		//DAC full scale current
//		DDS_addr = 0x040C;
//		DDS_data_size = 1;
//		DDS_buff[0] = 0x00;
//		write_DDS(DDS_addr, DDS_buff, DDS_data_size);
//		//tim3_delay();
//		
//		//DAC full scale current
//		DDS_addr = 0x040B;
//		DDS_data_size = 1;
//		DDS_buff[0] = 0xFF;
//		write_DDS(DDS_addr, DDS_buff, DDS_data_size);
//		//tim3_delay();
		DDS_stb_write(DDS.STB);
		
		//register update
		DDS_addr = 0x05;
		DDS_data_size = 1;
		DDS_buff[0] = 0x01;
		write_DDS(DDS_addr, DDS_buff, DDS_data_size);
		//tim3_delay();
	
		SPD.SPD_status_reg &= ~SPD_DDS_err_flag ;	
		return 1;
	}
	
	SPD.SPD_status_reg |= SPD_DDS_err_flag ;
	return 0;

}

void sleep_DDS(void)
{
	HAL_GPIO_WritePin(SPI4_PWDN_GPIO_Port, SPI4_PWDN_Pin, GPIO_PIN_SET);
	//tim3_delay();
	DDS.pwrdwn = 1;
}

void wakeup_DDS(void)
{
	HAL_GPIO_WritePin(SPI4_PWDN_GPIO_Port, SPI4_PWDN_Pin, GPIO_PIN_RESET);
	//tim3_delay();
	DDS.pwrdwn = 0;
}

uint8_t DDS_stb_write(uint16_t inp_code)
{
	if(DET.IsPowerOn == 0)
		return 0;
	
	uint8_t resp = 0;
	uint16_t DDS_addr = 0x0 ;
	uint8_t DDS_buff[2] = {0x0, 0x0};
	uint8_t DDS_data_size = 1;
	
	uint16_t code = (uint16_t)(inp_code & 0x3FF);
	DDS_buff[0]	= (uint8_t)((code & 0x300) >> 8);
	DDS_buff[1]	= (uint8_t)( code & 0xFF );
	
	//DAC full scale current
	DDS_addr = 0x040C;
	DDS_data_size = 1;
	write_DDS(DDS_addr, &DDS_buff[0], DDS_data_size);
	//tim3_delay();
	
	DDS_addr = 0x040B;
	DDS_data_size = 1;
	write_DDS(DDS_addr, &DDS_buff[1], DDS_data_size);
	//tim3_delay();
	
	//register update
	DDS_addr = 0x05;
	DDS_data_size = 1;
	DDS_buff[0] = 0x01;
	write_DDS(DDS_addr, DDS_buff, DDS_data_size);
	//tim3_delay();
		
//	DDS.STB = inp_code;
	return 1;
	
}

uint8_t DDS_stb_read(uint16_t *outp_code)
{
	if(DET.IsPowerOn == 0)
		return 0;
	
	uint8_t resp = 0;
	uint16_t DDS_addr = 0x0 ;
	uint8_t DDS_buff[2] = {0x0, 0x0};
	uint8_t DDS_data_size = 1;
	
	//DAC full scale current
	DDS_addr = 0x040C;
	DDS_data_size = 1;
	DDS_buff[0] = 0;
	read_DDS(DDS_addr, &DDS_buff[0], DDS_data_size);
	//tim3_delay();
	
	DDS_addr = 0x040B;
	DDS_data_size = 1;
	DDS_buff[1] = 0;
	read_DDS(DDS_addr, &DDS_buff[1], DDS_data_size);
	
	*outp_code = (uint16_t)( (DDS_buff[0] << 8) | DDS_buff[1] );
	DDS.STB = *outp_code ;
	
	return 1;
	
}

uint8_t DDS_freq_prms(void)
{
	if(DET.IsPowerOn == 0)
		return 0;
	
//	uint8_t resp = 0;
//	uint16_t DDS_addr = 0x0 ;
//	uint8_t DDS_buff[2] = {0x0, 0x0};
//	uint8_t DDS_data_size = 1;
//	
//	//DAC full scale current
//	DDS_addr = 0x040C;
//	DDS_data_size = 1;
//	DDS_buff[0] = 0;
//	read_DDS(DDS_addr, &DDS_buff[0], DDS_data_size);
//	//tim3_delay();
//	
//	DDS_addr = 0x040B;
//	DDS_data_size = 1;
//	DDS_buff[1] = 0;
//	read_DDS(DDS_addr, &DDS_buff[1], DDS_data_size);
	
	return 1;
}