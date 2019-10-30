#include "fifo_buffer.h"

void buffer_init(FIFO_BUFF * buff)
{
	buff->Tail = 0;
	buff->Head = 0;
	buff->Count = 0;
	buff->Skipped = 0;
	buff->Size = SIZE;
}

void Clear(FIFO_BUFF * buff)
{
	buff->Tail = 0;
	buff->Head = 0;
	buff->Skipped = 0;
}

uint32_t GetSize(FIFO_BUFF * buff)
{
	return ( buff->Size );
}


uint32_t IsEmpty(FIFO_BUFF * buff)
{
	return (buff->Head == buff->Tail); 

}


uint32_t GetAvailableSize(FIFO_BUFF * buff) 
{ 
	return (GetSize(buff) - GetPendingSize(buff)); 
}

uint32_t GetPendingSize(FIFO_BUFF * buff)
{
	return ( ((buff->Tail) >= (buff->Head)) ? ((buff->Tail) - (buff->Head)) : ((buff->Size) - (buff->Head) + (buff->Tail)));
}

uint32_t Put(uint32_t Data, FIFO_BUFF * buff)
{
	
	
	
		uint32_t NewTail = buff->Tail+1;
	
	
		if (NewTail == buff->Size)
			
			NewTail = 0;
		
	
		
		if (NewTail == buff->Head)
		{
		#ifdef OVERWRITE
			if ( (++buff->Head) >= (buff->Size))
			{
				buff->Head = 0;
	
				buff->Buffer[buff->Tail] = Data;
				buff->Tail = NewTail;
				return 0;
			}
		#else
	
			#ifdef USE_SKIPPED
				buff->Skipped++;
			#endif //USE_SKIPPED_COUNT
			return 0;
		#endif
			
		}
	
		buff->Buffer[buff->Tail] = Data;
		buff->Tail = NewTail;
		return 1;
}


uint32_t Get(uint32_t* Data, FIFO_BUFF * buff)
	{
		if (buff->Tail == buff->Head)
			return 0;
		*Data = buff->Buffer[buff->Head];
		if ( ++(buff->Head) >= (buff->Size) )
			buff->Head = 0;
		return 1;
	}
	
uint32_t Probe(uint32_t* Data, FIFO_BUFF * buff)
{
	*Data = buff->Buffer[buff->Head];
	return 1;
}	

