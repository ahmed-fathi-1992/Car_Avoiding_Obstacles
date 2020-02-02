/*
 * CFile1.c
 *
 * Created: 17/01/2020 15:43:03
 *  Author: ahmed_pc
 */ 


#include "gpio.h"

/**
 * Description: set port direction (which is DDR register)
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 *
 * @param direction: set the port direction and takes the following values
 * 				- 0x00 -> Input
 * 				- 0xff -> Output
 */
void gpioPortDirection(uint8_t u8_port, uint8_t u8_direction)
{
	switch (u8_port)
	{
		case GPIOA:
		PORTA_DIR = u8_direction;
		break;
		case GPIOB:
		PORTB_DIR = u8_direction;
		break;
		case GPIOC:
		PORTC_DIR = u8_direction;
		break;
		case GPIOD:
		PORTD_DIR = u8_direction;
		break;
		default:
		break;
	}
	
}

/**
 * Description: set the port value (which is PORT register)
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 * @param value: set the port value and takes the following values
 * 				- 0x00 -> Low
 * 				- 0xff -> High
 */
void gpioPortWrite(uint8_t u8_port, uint8_t u8_value)
{
		switch (u8_port)
		{
			case 0:
			PORTA_DATA = u8_value;
			break;
			case 1:
			PORTB_DATA = u8_value;
			break;
			case 2:
			PORTC_DATA = u8_value;
			break;
			case 3:
			PORTD_DATA = u8_value;
			break;
			default:
			break;
		}
	
}

/**
 * Description: toggle the port value (which is PORT register)
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 */
void gpioPortToggle(uint8_t u8_port)
{
			switch (u8_port)
			{
				case 0:
				PORTA_DATA = ~PORTA_DATA;
				break;
				case 1:
				PORTB_DATA = ~PORTB_DATA;
				break;
				case 2:
				PORTC_DATA = ~PORTC_DATA;
				break;
				case 3:
				PORTD_DATA = ~PORTD_DATA;
				break;
				default:
				break;
			}
	
	
}

/**
 * Description: read the current port value
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 * @return
 */
uint8_t gpioPortRead(uint8_t u8_port)
{
				switch (u8_port)
				{
					case 0:
					return PORTA_PIN;
					break;
					case 1:
					return PORTB_PIN;
					break;	
					case 2:
					return PORTC_PIN;
					break;
					case 3:
					return PORTD_PIN;
					break;
					default:
					return 0;
					break;
				}
	
	
}

/*===========================PIN Control===============================*/
/**
 * Description: set selected pins (more than one pin [ORed]) direction
 *
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 * @param pins: the selected pins the user need to set its direction and takes
 * 				any member from the enum EN_bits:
 * 				- BIT0
 * 				- BIT1
 * 				- BIT2
 * 				- BIT3
 * 				- BIT4
 * 				- BIT5
 * 				- BIT6
 * 				- BIT7
 *
 * @param direction: set the pins direction and takes the following values
 * 				- 0x00 -> Input
 * 				- 0xff -> Output
 */
void gpioPinDirection(uint8_t u8_port, uint8_t u8_pins, uint8_t u8_direction)
{
		switch (u8_port)
		{
			case 0:
			PORTA_DIR = PORTA_DIR & (~u8_pins);
			PORTA_DIR = PORTA_DIR |(u8_pins & u8_direction);
			break;
			case 1:
			PORTB_DIR = PORTB_DIR & (~u8_pins);
			PORTB_DIR = PORTB_DIR |(u8_pins & u8_direction);
			break;
			case 2:
			PORTC_DIR = PORTC_DIR & (~u8_pins);
			PORTC_DIR = PORTC_DIR |(u8_pins & u8_direction);
			break;
			case 3:
			PORTD_DIR = PORTD_DIR & (~u8_pins);
			PORTD_DIR = PORTD_DIR |(u8_pins & u8_direction);
			break;
			default:
			break;
		}
	
	
}

/**
 * Description: set selected pins (more than one pin [ORed]) values
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 * @param pins: the selected pins the user need to write its values and takes
 * 				any member from the enum EN_bits:
 * 				- BIT0
 * 				- BIT1
 * 				- BIT2
 * 				- BIT3
 * 				- BIT4
 * 				- BIT5
 * 				- BIT6
 * 				- BIT7
 * @param value
 */
void gpioPinWrite(uint8_t u8_port, uint8_t u8_pins, uint8_t u8_value)
{
			switch (u8_port)
			{
				case 0:
				PORTA_DATA = PORTA_DATA & (~u8_pins);
				PORTA_DATA = PORTA_DATA |(u8_pins & u8_value);
				break;
				case 1:
				PORTB_DATA = PORTB_DATA & (~u8_pins);
				PORTB_DATA = PORTB_DATA |(u8_pins & u8_value);
				break;
				case 2:
				PORTC_DATA = PORTC_DATA & (~u8_pins);
				PORTC_DATA = PORTC_DATA |(u8_pins & u8_value);
				break;
				case 3:
				PORTD_DATA = PORTD_DATA & (~u8_pins);
				PORTD_DATA = PORTD_DATA |(u8_pins & u8_value);
				break;
				default:
				break;
			}

	
	
}

/**
 * Description: toggle selected pin (only one pin) direction
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 * @param pins: the selected pin the user need to toggle it's value and takes
 * 				only one member from the enum EN_bits:
 * 				- BIT0
 * 				- BIT1
 * 				- BIT2
 * 				- BIT3
 * 				- BIT4
 * 				- BIT5
 * 				- BIT6
 * 				- BIT7
 */
void gpioPinToggle(uint8_t u8_port, uint8_t u8_pins)
{
				switch (u8_port)
				{
					case 0:
					PORTA_DATA = PORTA_DATA ^ u8_pins;
					break;
					case 1:
					PORTB_DATA = PORTB_DATA ^ u8_pins;
					break;
					case 2:
					PORTC_DATA = PORTC_DATA ^ u8_pins;
					break;
					case 3:
					PORTD_DATA = PORTD_DATA ^ u8_pins;
					break;
					default:
					break;
				}
	
	
}

/**
 * Description: read selected pin (only one pin) direction
 * @param port: portID and takes the following values
 * 				- 0 -> GPIOA
 * 				- 1 -> GPIOB
 * 				- 2 -> GPIOC
 * 				- 3 -> GPIOD
 *
 * @param pin: the selected pin the user need to read it's value and takes
 * 				only one member from the enum EN_bits:
 * 				- BIT0
 * 				- BIT1
 * 				- BIT2
 * 				- BIT3
 * 				- BIT4
 * 				- BIT5
 * 				- BIT6
 * 				- BIT7
 * @return
 */
uint8_t gpioPinRead(uint8_t u8_port, uint8_t u8_pin)
{
	
					switch (u8_port)
					{
						case 0:
						if(PORTA_PIN &u8_pin)
						return 1;
						else
						return 0;
						break;
						case 1:
						if(PORTB_PIN &u8_pin)
						return 1;
						else
						return 0;
						break;
						case 2:
						if(PORTC_PIN &u8_pin)
						return 1;
						else
						return 0;
						break;
						case 3:
						if(PORTD_PIN &u8_pin)
						return 1;
						else
						return 0;
						break;
						default:
						return 0;
						break;
					}

	
}



