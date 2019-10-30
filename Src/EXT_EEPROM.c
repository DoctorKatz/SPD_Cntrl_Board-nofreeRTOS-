#include "EXT_EEPROM.h"
#include "stddef.h"

volatile uint32_t eeprom_buffer [eeprom_buffer_size];	
extern I2C_HandleTypeDef hi2c3;
extern CRC_HandleTypeDef hcrc;

//eeprom procedures
uint8_t eeprom_test(void);
uint8_t eeprom_write (uint32_t start_addr, uint8_t *data);
uint8_t eeprom_read (uint32_t start_addr, uint8_t *data);
uint8_t copy_to_eeprom(uint32_t  addr_dest, void * ptr_source, uint16_t size_source);
uint8_t copy_from_eeprom(uint32_t  addr_source, void * ptr_dest, uint16_t size_source);


uint8_t eeprom_write (uint32_t st_addr, uint8_t *data)
{
	HAL_GPIO_WritePin(GPIOC, TP_I2C3_Pin, GPIO_PIN_SET);
	uint16_t dev_sel = 0;
	uint8_t data_send[3];
	uint32_t start_addr = (uint32_t)st_addr;
	HAL_StatusTypeDef write_status = HAL_OK;
	
	start_addr &= 0x3FFFF;
	dev_sel = 0xA0;
	dev_sel |= (start_addr & 0x30000)>>(16-1);
	data_send[0] = (uint8_t)((start_addr & 0xFF00) >> 8);
	data_send[1] = (uint8_t)(start_addr & 0xFF);
	data_send[2] = *data;
	
	HAL_GPIO_WritePin(GPIOC, EEPROM_WC_Pin, GPIO_PIN_RESET);
	write_status = HAL_I2C_Mem_Write(&hi2c3, dev_sel, start_addr, 2, &data_send[2], 1, 10);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, EEPROM_WC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, TP_I2C3_Pin, GPIO_PIN_RESET);
	if (write_status==HAL_OK)
		return 1;
	else
		return 0;
}

uint8_t eeprom_read (uint32_t st_addr, uint8_t *data)
{
	HAL_GPIO_WritePin(GPIOC, TP_I2C3_Pin, GPIO_PIN_SET);
	uint8_t dev_sel;
	uint8_t data_out;
	uint32_t start_addr = (uint32_t)st_addr;
	HAL_StatusTypeDef status_rd = HAL_OK;
	
	HAL_GPIO_WritePin(GPIOC, EEPROM_WC_Pin, GPIO_PIN_SET);
	
	start_addr &= 0x3FFFF;
	dev_sel = 0xA0;
	dev_sel |= (start_addr & 0x30000)>>(16-1);
	dev_sel |=0x1;//установка режима чтения
	status_rd = HAL_I2C_Mem_Read(&hi2c3, dev_sel, start_addr, 2, &data_out, 1, 1000);
	HAL_GPIO_WritePin(GPIOC, TP_I2C3_Pin, GPIO_PIN_RESET);
	if(status_rd != HAL_OK)
	{
		*data = 0;
		return 0;
	}
	*data = data_out;
	return 1;
}


uint8_t copy_to_eeprom(uint32_t  addr_dest, void * ptr_source, uint16_t size_source)
{
		HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
		uint8_t status = 0;//результат выполения операции
		

		uint32_t CRC32_accum_wr = 0;//записываемый CRC
		uint32_t CRC32_accum_rd = 0;//считываемый CRC
		uint32_t CRC32_accum_cl = 0;//вычисленный CRC
//		uint8_t eeprom_data_wr = 0x00;
		uint8_t eeprom_data_rd = 0x00;
		uint32_t eeprom_addr = 0xFFFF;
		uint32_t ii, j, mask, size_CRC = 4;
		uint8_t temp;
		uint32_t CRC_temp;
		
		//uint32_t *ptr_mem;
		//ptr_mem = (uint32_t*)eeprom_buffer;				
		//void * ptr_src = (void*)ptr_src;
		void *ptr_mem = NULL;
		//ptr_mem = (uint8_t*)eeprom_buffer;				
		
		HAL_CRC_StateTypeDef state_CRC;
				
		memcpy ( (void*)eeprom_buffer, (void*)ptr_source,  size_source);//размер памяти в байтах
		
		//ptr_mem = (uint8_t*)eeprom_buffer;
		ptr_mem = (void*)eeprom_buffer;
		//вычисление CRC кода данных до записи
		__HAL_CRC_DR_RESET(&hcrc);
		state_CRC = HAL_CRC_GetState (&hcrc);
		if (state_CRC==HAL_CRC_STATE_READY)
		{
			for (ii = 0; ii < size_source; ii++)
			{		
				CRC_temp = (uint32_t)(*(uint8_t*)ptr_mem);
				CRC_temp &= 0xFF;
				CRC32_accum_wr = HAL_CRC_Accumulate(&hcrc, &CRC_temp, 1);
				ptr_mem = (uint8_t*)ptr_mem + 1;
				while (HAL_CRC_GetState(&hcrc)==HAL_CRC_STATE_BUSY) ;
			}
		}
		
		//байтовая запись данных в память
		ptr_mem = (void*)eeprom_buffer;
		for (ii = 0; ii < size_source; ii++)
		{
			eeprom_addr = addr_dest + ii;
			eeprom_write(eeprom_addr, ptr_mem);
			ptr_mem = (uint8_t*)ptr_mem + 1;
			HAL_Delay(10);
		}
		
		//запись CRC32 кода в EEPROM
		ii++;
		for (j = 0; j < size_CRC; j++)
		{
				eeprom_addr = addr_dest + ii + j;
				mask = 0xFF << (8 * j);
				temp = (uint8_t)((CRC32_accum_wr & mask) >> (8 * j));
				eeprom_write(eeprom_addr, &temp);
				HAL_Delay(10);
		}
		
		ptr_mem = (void*)eeprom_buffer;
		//байтовое чтение из памяти в буфер
		for (ii = 0; ii < size_source; ii++)
		{
				eeprom_addr = addr_dest + ii;
				eeprom_read(eeprom_addr, &eeprom_data_rd);
				*(uint8_t*)ptr_mem = eeprom_data_rd;
				ptr_mem = (uint8_t*)ptr_mem + 1;
		}

		//чтение CRC32 кода из EEPROM
		ii++;
		CRC32_accum_rd = 0;
		for (j = 0; j < size_CRC; j++)
		{
			eeprom_addr = addr_dest + ii + j;
			eeprom_read(eeprom_addr, &temp);
			CRC32_accum_rd |= (uint32_t)(temp << (8 * j));
		}		
		
		//вычисляем CRC прочитанных данных
		ptr_mem = (void*)eeprom_buffer;
		__HAL_CRC_DR_RESET(&hcrc);
		state_CRC = HAL_CRC_GetState (&hcrc);
		//вычисляем CRC, трактуя байты как uint8
		if (state_CRC==HAL_CRC_STATE_READY)
		{
			for (ii = 0; ii < size_source; ii++)
			{		
				CRC_temp = (uint32_t)(*(uint8_t*)ptr_mem);
				CRC_temp &= 0xFF;
				CRC32_accum_cl = HAL_CRC_Accumulate(&hcrc, &CRC_temp, 1);
				ptr_mem = (uint8_t*)ptr_mem + 1;
				while (HAL_CRC_GetState (&hcrc)==HAL_CRC_STATE_BUSY) ;
			}
		}
			
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
		
		if ((CRC32_accum_wr != CRC32_accum_rd) || (CRC32_accum_rd != CRC32_accum_cl))
		{
			status |= error_CRC_eeprom;
			return 0;//ошибка и выход
		}
		status = 1;
		return status;
			return 0;
}

uint8_t copy_from_eeprom(uint32_t  addr_source, void * ptr_destination, uint16_t size_source)
{
		HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
		
		uint32_t CRC32_accum_cl = 0;//вычисленнвй CRC
		uint32_t CRC32_accum_rd = 0;//считываемый CRC из EEPROM
				
		HAL_CRC_StateTypeDef state_CRC;
		
		//uint8_t eeprom_data_wr = 0x00;
		uint8_t eeprom_data_rd = 0x00;
		uint32_t eeprom_addr = 0xFFFF;
		//uint16_t eeprom_addr_adj = 1;
		uint32_t ii, mask;
		uint8_t temp, status = 0, size_CRC = 4;
		uint32_t CRC_temp;
	
		void *ptr_dest = (void*)eeprom_buffer;
		

		//байтовое чтение памяти
		for (ii = 0; ii < size_source; ii++)
		{
				eeprom_addr = addr_source + ii;
				eeprom_read(eeprom_addr, &eeprom_data_rd);
				*(uint8_t*)ptr_dest = eeprom_data_rd;
					//((uint8_t*)ptr_dest)++;
				ptr_dest = (uint8_t*)ptr_dest + 1;
		}

		//собираем байт из 4х, получаем uint32
		//чтение CRC32 кода из EEPROM
		ii++;
		CRC32_accum_rd = 0;
		for (uint32_t j = 0; j < size_CRC; j++)
		{
					eeprom_addr = addr_source + ii + j;
					eeprom_read(eeprom_addr, &temp);
					mask = 0xFF << (8 * j);
					CRC32_accum_rd = (((uint32_t)(temp << (8 * j))) & mask )| CRC32_accum_rd;
		}		
				
		//вычисляем CRC прочитанных данных
		ptr_dest = (void*)eeprom_buffer;
		__HAL_CRC_DR_RESET(&hcrc);
		state_CRC = HAL_CRC_GetState (&hcrc);
		//вычисляем CRC, трактуя байты как uint32
		if (state_CRC==HAL_CRC_STATE_READY)
		{
			for (ii = 0; ii < size_source; ii++)
			{		
					CRC_temp = (uint32_t)(*(uint8_t*)ptr_dest);
					CRC_temp &= 0xFF;
					CRC32_accum_cl = HAL_CRC_Accumulate(&hcrc, &CRC_temp, 1);
					ptr_dest = (uint8_t*)ptr_dest + 1;
					while (HAL_CRC_GetState (&hcrc)==HAL_CRC_STATE_BUSY) ;
			}
		}
				
		if (CRC32_accum_cl != CRC32_accum_rd)
		{
			status |= error_CRC_eeprom;
			return 0;//ошибка и выход
		}
		
		
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
		
		//если всё хорошо, тогда копируем из буфера в указанную область памяти
		//ptr_dest = (uint8_t*)eeprom_buffer;
		ptr_dest = (void*)eeprom_buffer;
		memcpy ( (void*)ptr_destination, (void*)ptr_dest, size_source );
	
		status = 1;
		return 1;//успех	
	return 0;
}


uint8_t eeprom_test(void)
{
	//проверка связи с EEPROM
	uint32_t mem_addr = 0xFFFF;
	uint8_t mem_data_write = 0xA5;
	uint8_t mem_data_read = 0x0;
	eeprom_write (mem_addr, &mem_data_write);
	HAL_Delay(10);
	eeprom_read (mem_addr, &mem_data_read);
	if(mem_data_write == mem_data_read)
	{
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_RESET);
		return 1;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD, GREEN_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, RED_LED_Pin, GPIO_PIN_SET);
		return 0;
	}	
}
	
	