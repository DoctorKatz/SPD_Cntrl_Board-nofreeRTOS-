#include "stm32f4xx_hal.h"

#define SIZE	32
#define OVERWRITE	1 
#define USE_SKIPPED 1

#define ADC_COUNT 1

#define ADC_SUB_CHANNELS 3

#define FIFO_COUNT (ADC_COUNT * ADC_SUB_CHANNELS)

	typedef struct 
	{
		uint32_t Tail;
		uint32_t Head;
		uint32_t Count;
		uint32_t	Size;
		uint32_t	Skipped;
		uint32_t Buffer[SIZE];
	} FIFO_BUFF;

void buffer_init(FIFO_BUFF*);
uint32_t GetSize(FIFO_BUFF*);
uint32_t IsEmpty(FIFO_BUFF*);
uint32_t GetPendingSize(FIFO_BUFF*);
uint32_t GetAvailableSize(FIFO_BUFF*);
void Clear(FIFO_BUFF*);
uint32_t Put(uint32_t, FIFO_BUFF*);
uint32_t Get(uint32_t*, FIFO_BUFF*);
uint32_t Probe(uint32_t*, FIFO_BUFF*);

