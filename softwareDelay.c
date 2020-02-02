/*
 * softwareDelay.c
 *
 * Created: 17/01/2020 20:20:42
 *  Author: ahmed_pc
 */ 

#include "softwareDelay.h "

void softwareDelayMs(uint32_t u32_delay_in_ms)
{
		volatile uint32_t d=0;
		for(int y=0;y<u32_delay_in_ms;y++)
		{			
			for(int z=0;z<19;z++)
			{
				for(int w=0;w<16;w++)
				{
					d++;					
				}	
				
			}
		}
		
	
	
}

void softwareDelayus(uint32_t u32_delay_in_us)
{
	volatile uint32_t d=0;
	for(int y=0;y<u32_delay_in_us;y++)
	{
		
		for(int z=0;z<1;z++)
		{
			d++;			
		}
	}	
}
