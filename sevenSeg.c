/*
 * sevenSeg.c
 *
 * Created: 17/01/2020 22:40:00
 *  Author: ahmed_pc
 */ 
#include "sevenSeg.h  "

void sevenSegInit(En_SevenSegId_t a_segment_id)
{
	gpioPinDirection(SEG_BCD_GPIO, SEG_BCD_BITS, OUTPUT);
	gpioPinWrite(SEG_BCD_GPIO, SEG_BCD_BITS, HIGH);
		switch (a_segment_id)
		{
			case SEG_0:
			gpioPinDirection(SEG_EN1_GPIO, SEG_EN1_BIT, OUTPUT);
			gpioPinWrite(SEG_EN1_GPIO, SEG_EN1_BIT, SEG_DISABLE);
			break;
			case SEG_1:
			gpioPinDirection(SEG_EN2_GPIO, SEG_EN2_BIT, OUTPUT);
			gpioPinWrite(SEG_EN2_GPIO, SEG_EN2_BIT, SEG_DISABLE);
			break;

			default:
			break;
		}
	
	
}

/**
 * Description: 
 * @param 
 */
void sevenSegEnable(En_SevenSegId_t en_segment_id)
{
			switch (en_segment_id)
			{
				case SEG_0:
				gpioPinWrite(SEG_EN1_GPIO, SEG_EN1_BIT, SEG_ENABLE);
				break;
				case SEG_1:
				gpioPinWrite(SEG_EN2_GPIO, SEG_EN2_BIT, SEG_ENABLE);
				break;

				default:
				break;
			}

	
	
}

/**
 * Description: 
 * @param 
 */
void sevenSegDisable(En_SevenSegId_t en_segment_id)
{
			switch (en_segment_id)
			{
				case SEG_0:
				gpioPinWrite(SEG_EN1_GPIO, SEG_EN1_BIT, SEG_DISABLE);
				break;
				case SEG_1:
				gpioPinWrite(SEG_EN2_GPIO, SEG_EN2_BIT, SEG_DISABLE);
				break;

				default:
				break;
			}

	
	
}

/**
 * Description: 
 * @param 
 */
void sevenSegWrite(En_SevenSegId_t en_segment_id, uint8_t u8_number)
{
	gpioPinWrite(SEG_BCD_GPIO, SEG_BCD_BITS, (HIGH & 0x0F));
	sevenSegEnable( en_segment_id);	
	gpioPinWrite(SEG_BCD_GPIO, SEG_BCD_BITS, (u8_number & 0x0F) );//(u8_number & 0x01)
}
