/*
 * dcMotor.c
 *
 * Created: 27/01/2020 16:57:28
 *  Author: ahmed_pc
 */ 
#include "dcMotor.h"
#include "HwPWM.h"


void MotorDC_Init(En_motorType_t en_motor_number)
{
	timer0Init(T0_NORMAL_MODE,T0_OC0_DIS,T0_PRESCALER_256, 0, 0, T0_INTERRUPT_NORMAL);
		switch (en_motor_number)
		{
			case MOT_1:
				gpioPinDirection(MOTOR_OUT_1A_GPIO, MOTOR_OUT_1A_BIT, OUTPUT);
				gpioPinDirection(MOTOR_OUT_1B_GPIO, MOTOR_OUT_1B_BIT, OUTPUT);
				gpioPinDirection(MOTOR_EN_1_GPIO, MOTOR_EN_1_BIT, OUTPUT);
				gpioPinWrite(MOTOR_OUT_1A_GPIO, MOTOR_OUT_1A_BIT, LOW);
				gpioPinWrite(MOTOR_OUT_1B_GPIO, MOTOR_OUT_1B_BIT, LOW);
				gpioPinWrite(MOTOR_EN_1_GPIO, MOTOR_EN_1_BIT, LOW);
	
			break;
			case MOT_2:
				gpioPinDirection(MOTOR_OUT_2A_GPIO, MOTOR_OUT_2A_BIT, OUTPUT);
				gpioPinDirection(MOTOR_OUT_2B_GPIO, MOTOR_OUT_2B_BIT, OUTPUT);
				gpioPinDirection(MOTOR_EN_2_GPIO, MOTOR_EN_2_BIT, OUTPUT);
				gpioPinWrite(MOTOR_OUT_2A_GPIO, MOTOR_OUT_2A_BIT, LOW);
				gpioPinWrite(MOTOR_OUT_2B_GPIO, MOTOR_OUT_2B_BIT, LOW);
				gpioPinWrite(MOTOR_EN_2_GPIO, MOTOR_EN_2_BIT, LOW);
				
			break;

			default:
			break;
		}
		
	
}

/**
 * Description: 
 * @param 
 * @param 
 */
void MotorDC_Dir(En_motorType_t en_motor_number, En_motorDir_t en_motor_dir)
{

	  switch (en_motor_number)
	  {
//////////motor1///////////
		  case MOT_1:
		  switch (en_motor_dir)
		  {
			  case FORWARD:
				gpioPinWrite(MOTOR_OUT_1A_GPIO, MOTOR_OUT_1A_BIT, HIGH);
				gpioPinWrite(MOTOR_OUT_1B_GPIO, MOTOR_OUT_1B_BIT, LOW);
			  break;
			  case BACKWARD:
				gpioPinWrite(MOTOR_OUT_1A_GPIO, MOTOR_OUT_1A_BIT, LOW);
				gpioPinWrite(MOTOR_OUT_1B_GPIO, MOTOR_OUT_1B_BIT, HIGH);
			 break;
			  case STOP:
				gpioPinWrite(MOTOR_OUT_1A_GPIO, MOTOR_OUT_1A_BIT, LOW);
				gpioPinWrite(MOTOR_OUT_1B_GPIO, MOTOR_OUT_1B_BIT, LOW);
			  break;
			  default:
			  break;
		  }
		  break;
		  
////////motor2//////////
		  case MOT_2:
		  switch (en_motor_dir)
		  {
			  case FORWARD:
				gpioPinWrite(MOTOR_OUT_2A_GPIO, MOTOR_OUT_2A_BIT, HIGH);
				gpioPinWrite(MOTOR_OUT_2B_GPIO, MOTOR_OUT_2B_BIT, LOW);
			  break;
			  case BACKWARD:
				gpioPinWrite(MOTOR_OUT_2A_GPIO, MOTOR_OUT_2A_BIT, LOW);
				gpioPinWrite(MOTOR_OUT_2B_GPIO, MOTOR_OUT_2B_BIT, HIGH);
			  break;
			  case STOP:
				gpioPinWrite(MOTOR_OUT_2A_GPIO, MOTOR_OUT_2A_BIT, LOW);
				gpioPinWrite(MOTOR_OUT_2B_GPIO, MOTOR_OUT_2B_BIT, LOW);
			  break;
			  default:
			  break;
		  }
			 break;
			  
		  default:
		  break;
	  }	

}

/**
 * Description: 
 * @param 
 */
void MotorDC_Speed_PollingWithT0(uint8_t u8_motor_speed)
{

	timer0SwPWM(u8_motor_speed,200,MOTOR_EN_1_BIT,MOTOR_EN_2_BIT);
	
}

/**
 * Description: set the port value (which is PORT register)
 * @param 
 */
void MotorDC_Speed_HwPWM(uint8_t u8_motor_speed)
{
  HwPWMSetDuty(u8_motor_speed, 200);	
}
