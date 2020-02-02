/*
 * CFile1.c
 *
 * Created: 17/01/2020 19:25:26
 *  Author: ahmed_pc
 */ 
#include "led.h"


/**
 * Description: Initialize the LED_x (where x 0, 1, 2, 3) as output
 * @param led_id: The led to be initialized and it takes
 * 				  one of the enum (En_LedNumber_t) parameters
 *
 */
void Led_Init(En_LedNumber_t en_led_id)
{
		switch (en_led_id)
		{
			case 0:
			gpioPinDirection(LED_0_GPIO, LED_0_BIT, OUTPUT);
			gpioPinWrite(LED_0_GPIO, LED_0_BIT, LOW);
			break;
			case 1:
			gpioPinDirection(LED_1_GPIO, LED_1_BIT, OUTPUT);
			gpioPinWrite(LED_1_GPIO, LED_1_BIT, LOW);
			break;
			case 2:
			gpioPinDirection(LED_2_GPIO, LED_2_BIT, OUTPUT);
			gpioPinWrite(LED_2_GPIO, LED_2_BIT, LOW);
			break;
			case 3:
			gpioPinDirection(LED_3_GPIO, LED_3_BIT, OUTPUT);
			gpioPinWrite(LED_3_GPIO, LED_3_BIT, LOW);
			break;
			default:
			break;
		}

	
	
}
/**
 * Description: Change the LED_x state (where x 0, 1, 2, 3) to be on
 * @param led_id: The led to be initialized and it takes
 * 				  one of the enum (En_LedNumber_t) parameters
 *
 */
void Led_On(En_LedNumber_t en_led_id)
{
			switch (en_led_id)
			{
				case 0:
				gpioPinWrite(LED_0_GPIO, LED_0_BIT, HIGH);
				break;
				case 1:
				gpioPinWrite(LED_1_GPIO, LED_1_BIT, HIGH);
				break;
				case 2:
				gpioPinWrite(LED_2_GPIO, LED_2_BIT, HIGH);
				break;
				case 3:
				gpioPinWrite(LED_3_GPIO, LED_3_BIT, HIGH);
				break;
				default:
				break;
			}
	
	
}
/**
 * Description: Change the LED_x state (where x 0, 1, 2, 3) to be off
 * @param led_id: The led to be initialized and it takes
 * 				  one of the enum (En_LedNumber_t) parameters
 *
 */
void Led_Off(En_LedNumber_t en_led_id)
{
				switch (en_led_id)
				{
					case 0:
					gpioPinWrite(LED_0_GPIO, LED_0_BIT, LOW);
					break;
					case 1:
					gpioPinWrite(LED_1_GPIO, LED_1_BIT, LOW);
					break;
					case 2:
					gpioPinWrite(LED_2_GPIO, LED_2_BIT, LOW);
					break;
					case 3:
					gpioPinWrite(LED_3_GPIO, LED_3_BIT, LOW);
					break;
					default:
					break;
				}

	
	
}
/**
 * Description: Toggle the LED_x state (where x 0, 1, 2, 3)
 * @param led_id: The led to be initialized and it takes
 * 				  one of the enum (En_LedNumber_t) parameters
 *
 */
void Led_Toggle(En_LedNumber_t en_led_id)
{
				switch (en_led_id)
				{
					case 0:					
					gpioPinToggle(LED_0_GPIO, LED_0_BIT);
					break;
					case 1:
					gpioPinToggle(LED_1_GPIO, LED_1_BIT);
					break;
					case 2:
					gpioPinToggle(LED_2_GPIO, LED_2_BIT);
					break;
					case 3:
					gpioPinToggle(LED_3_GPIO, LED_3_BIT);
					break;
					default:
					break;
				}

	
	
}
