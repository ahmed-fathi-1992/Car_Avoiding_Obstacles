/*
 * HwPWM.c
 *
 * Created: 02/02/2020 19:30:23
 *  Author: ahmed_pc
 */ 

#include "HwPWM.h"
#include "dcMotor.h"
void HwPWMInit(void)
{
	//initialize timer 1 to be in HWpwm mode  
  timer1Init(T1_Fast_PWM_8_bit,(T1_OC1A_CLEAR | T1_OC1B_CLEAR ),T1_PRESCALER_256, 0, 0, 0,0,T1_POLLING);
  
	gpioPinDirection(MOTOR_EN_1_GPIO, MOTOR_EN_1_BIT, OUTPUT); // configure motor1 enable pin to be output
	gpioPinDirection(MOTOR_EN_2_GPIO, MOTOR_EN_2_BIT, OUTPUT); // configure motor2 enable pin to be output
	
}

void HwPWMSetDuty(uint8_t a_u8_duty, uint32_t a_u32_frequency)
{	
	OCR1A = (a_u8_duty*255)/100; //set OCR1A to duty cycle 
	OCR1B = (a_u8_duty*255)/100;//set OCR1A to duty cycle 	
}
