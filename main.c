/*
 * gpio.c
 *
 * Created: 17/01/2020 14:44:58
 *  Author: ahmed_pc
 */ 

#define Triger_pin 0x2


typedef enum prog {
  seg_counter,
  Button_press,
  Traffic_light,
  Motor,
  ultrasonic,
  Car,
  Test,
}prog;

typedef enum{
	GO_state=0,Stop_state,Get_Ready_state
}prog_states;

void seg_counter_fun(void);
void Button_press_fun(void);
void Traffic_light_fun(void);
void MOTOR(void);
void Ultrasonic(void);
void Car_Avoiding_Obstacles(void);
void Test_1(void);

//#include <avr/io.h>
#include "registers.h"
#include "gpio.h"
#include "led.h"
#include "softwareDelay.h"
#include "pushButton.h"
#include "sevenSeg.h"
#include "timers.h"
#include "interrupt.h"
#include "std_types.h"
#include "dcMotor.h"
#include "SwICU.h"
#include "HwPWM.h"

uint32_t spwm_duty_cycle=0;
EN_bits_t spwm_pin_1= MOTOR_EN_1_BIT;
EN_bits_t spwm_pin_2= MOTOR_EN_2_BIT;

volatile uint8_t Eco_width=7;
volatile EN_SwICU_Edge_t Edge=SwICU_EdgeRisiging;

volatile uint32_t testing=0;


int main(void)
{
	prog program = Car; // select program to run GPIO_REQ7:(seg_counter) GPIO_REQ8:(Button_press) GPIO_REQ9:(Traffic_light)
	                           // GPIO_REQ15:(MOTOR)  Ultrasonic 15cm range :ultrasonic Car Avoiding Obstacles(REQ4): Car
 switch (program)
  {
		case seg_counter:
		 seg_counter_fun();				
		break;
		
		case Button_press:
		 Button_press_fun();
		break;
		
		case Traffic_light:
		 Traffic_light_fun();		
		break;
		
		case Motor:
		MOTOR();
		break;
		
		case ultrasonic:
		Ultrasonic();
		break;
		
		case Car:
		Car_Avoiding_Obstacles();
		break;
		
		case Test:
		Test_1();
		break;

		default:
		break;
 }
	
}


	
void seg_counter_fun(void)
{
			sevenSegInit(SEG_0);
			sevenSegInit(SEG_1);
			while(1)
			{
				//softwareDelayMs(1000);
				timer0DelayMs(1000);
				for (int digit_1 =0 ;digit_1<10 ;digit_1++)
				{
					for(int digit_0 =0 ;digit_0<10 ;digit_0++)
					{
						for (int count =0 ;count<100 ;count++)
						{
							
							sevenSegWrite(SEG_0, digit_0);
							timer0DelayMs(5);
							//softwareDelayMs(5);
							sevenSegDisable( SEG_0);
							
							sevenSegWrite(SEG_1, digit_1);
							timer0DelayMs(5);
							//softwareDelayMs(5);
							sevenSegDisable( SEG_1);
						}
					}
				}

			}	
}

void Button_press_fun(void)
{
	uint8_t led_on_repeat=0;
	timer0Init(T0_NORMAL_MODE,T0_OC0_DIS,T0_PRESCALER_64, 0, 0, T0_INTERRUPT_NORMAL);
	pushButtonInit(BTN_1);
	Led_Init(LED_1);
	while(1)
	{  led_on_repeat=0;
		if( Prepressed== pushButtonGetStatus(BTN_1))
		{
			while(Prepressed== pushButtonGetStatus(BTN_1));
			Led_On(LED_1);
			for (int counter=0; counter<20;counter++)
			{
				if( Prepressed== pushButtonGetStatus(BTN_1))
				led_on_repeat++;
												
			}
			if( led_on_repeat>0) {timer0DelayMs(1000);} //softwareDelayMs(1000);
			
			Led_Off(LED_1);
			
		}
		
		
	}
	
}

void Traffic_light_fun(void)
{
	prog_states state = GO_state;
	timer0Init(T0_NORMAL_MODE,T0_OC0_DIS,T0_PRESCALER_64, 0, 0, T0_INTERRUPT_NORMAL);
	Led_Init(LED_1); Led_Init(LED_2); Led_Init(LED_3);
	Led_Off(LED_1);  Led_Off(LED_2);  Led_Off(LED_3);
	//	Led_On(LED_1);   Led_On(LED_2);   Led_On(LED_3);
	
	while(1){
		switch(state) {
			case GO_state:        {Led_On(LED_1); timer0DelayMs(1000); Led_Off(LED_1);     state=Stop_state;       break;}
			case Stop_state:      {Led_On(LED_3); timer0DelayMs(1000); Led_Off(LED_3);     state=Get_Ready_state;  break;}
			case Get_Ready_state: {Led_On(LED_2); timer0DelayMs(1000); Led_Off(LED_2);     state=GO_state;         break;}
		}//end of switch.
	}//end of while	
	
}


void MOTOR(void)
{

int speed=0;

MotorDC_Init(MOT_1);
MotorDC_Init(MOT_2);
timer2Init(T0_NORMAL_MODE,T0_OC0_DIS,T0_PRESCALER_64, 0, 0, 0,T0_POLLING); // for delay

SREG= SREG |0x80 ;// enable global interrupt

	
	MotorDC_Dir(MOT_1, FORWARD);
	MotorDC_Dir(MOT_2, FORWARD);
	
	for (speed=0; speed<100;speed++)  // 5 second from %0 to %100
	{
		timer2DelayMs(50);
		MotorDC_Speed_PollingWithT0(speed);
	}
	for (speed=100; speed>0;speed--) // 5 second from %100 to %0
	{
		timer2DelayMs(50);
		MotorDC_Speed_PollingWithT0(speed);
	}
	
		MotorDC_Dir(MOT_1, BACKWARD);  //Motor_direction(Right);
		MotorDC_Dir(MOT_2, FORWARD);

	
	for (speed=0; speed<85;speed++) // .5 second to turn right 
	{
		timer2DelayMs(10);
		MotorDC_Speed_PollingWithT0(25);
	}


	MotorDC_Dir(MOT_1, STOP);
	MotorDC_Dir(MOT_2, STOP);
	
while(1);
//{}


}


		
void Ultrasonic(void)
{
  uint8_t distance=0;
  
  SwICU_Init(SwICU_EdgeRisiging);
  
 gpioPinDirection(GPIOA,  0x2, OUTPUT);
 	
  Led_Init(LED_0); Led_Init(LED_1); Led_Init(LED_2); Led_Init(LED_3);
  Led_Off(LED_0); Led_Off(LED_1); Led_Off(LED_2); Led_Off(LED_3);

  SwICU_Start();
  
   SREG= SREG |0x80 ;// enable global interrupt
   
  while(1)
  {
	  //send triger puls
	gpioPinWrite(GPIOA,  0x2, HIGH);
	softwareDelayMs(1);
	gpioPinWrite(GPIOA,  0x2, LOW);
	
	distance= (Eco_width*16)/58; //calculating distance in cm
	if ((TIFR & 0x40) || distance > 15) //make sure range in 15 cm
	{
		gpioPinWrite(GPIOB, 0xF0, HIGH)	;// all LEDs On	
		TIFR = TIFR |0x40;	//clear over flow flag 	    

	}else
	{
		   
		    PORTB_DATA= distance<<4; // show distance in binary on 4 leds 		
	}
	

	softwareDelayMs(600); //to avoid flickering 
  }	
}


void Car_Avoiding_Obstacles(void)
{
	  uint8_t distance=0;
	  
	  gpioPinDirection(GPIOA, Triger_pin, OUTPUT);
	  
	  Led_Init(LED_0); Led_Init(LED_1); Led_Init(LED_2); Led_Init(LED_3);
	  Led_Off(LED_0); Led_Off(LED_1); Led_Off(LED_2); Led_Off(LED_3);
	  
	  SwICU_Init(SwICU_EdgeRisiging);
	 
	  SwICU_Start();
	 
	  HwPWMInit();
	 
	        MotorDC_Init(MOT_1);
	        MotorDC_Init(MOT_2);
	        
	         MotorDC_Speed_HwPWM(80); // set duty cycle to 80%
	        
	  SREG= SREG |0x80 ;// enable global interrupt
	  
	  while(1)
	  {
		  //send triger puls
		  gpioPinWrite(GPIOA,  Triger_pin, HIGH);
		  softwareDelayMs(1);
		  gpioPinWrite(GPIOA,  Triger_pin, LOW);
		  
		  distance= (Eco_width*16)/58; //calculating distance in cm
		  if ((TIFR & 0x40) || distance > 15) //make sure range in 15 cm
		  {
			  gpioPinWrite(GPIOB, 0xF0, HIGH)	;// all LEDs On
			  TIFR = TIFR |0x40;	//clear over flow flag

		  }else
		  {
			  
			  PORTB_DATA= distance<<4; // show distance in binary on 4 leds
		  }
		  
		  
		  if (distance > 12) //make sure it will stop on 5 cm (try and error)
		  {
			  	MotorDC_Dir(MOT_1, FORWARD);
			 	MotorDC_Dir(MOT_2, FORWARD);

			  
		  }else
		  {
			  	MotorDC_Dir(MOT_1, STOP);
			  	MotorDC_Dir(MOT_2, STOP);
			  
		  }
		  
	 

		  softwareDelayMs(100); //to avoid flickering
	
	
    }
  }
void Test_1(void)
{
//MotorDC_Dir(MOT_2, STOP);	
      MotorDC_Init(MOT_1);
      MotorDC_Init(MOT_2);
      
      // MotorDC_Speed_HwPWM(80);
      			  	MotorDC_Dir(MOT_1, FORWARD);
      			  	MotorDC_Dir(MOT_2, FORWARD);
      gpioPinWrite(MOTOR_EN_1_GPIO, MOTOR_EN_1_BIT, HIGH);
      gpioPinWrite(MOTOR_EN_2_GPIO, MOTOR_EN_2_BIT, HIGH);
	  while(1);

}


ISR(TIMER0_OVF_vector)
{
	static volatile uint8_t flag =0;
	if(spwm_duty_cycle==0)
	{
		gpioPinWrite(MOTOR_EN_1_GPIO, spwm_pin_1, LOW);
		gpioPinWrite(MOTOR_EN_2_GPIO, spwm_pin_2, LOW);

	}else
	{
		if (spwm_duty_cycle==100)
		{
	      gpioPinWrite(MOTOR_EN_1_GPIO, spwm_pin_1, HIGH);
	      gpioPinWrite(MOTOR_EN_2_GPIO, spwm_pin_2, HIGH);
		}else
		{
			if(flag==0)
			{
			    gpioPinWrite(MOTOR_EN_1_GPIO, spwm_pin_1, HIGH);
		        gpioPinWrite(MOTOR_EN_2_GPIO, spwm_pin_2, HIGH);
				TCNT0=55+(200 - (spwm_duty_cycle*2));
				flag=1;
			}else
			{
			    gpioPinWrite(MOTOR_EN_1_GPIO, spwm_pin_1, LOW);
			    gpioPinWrite(MOTOR_EN_2_GPIO, spwm_pin_2, LOW);
				TCNT0=55+(spwm_duty_cycle*2);
				flag=0;
			}

			
		}
	}

}



ISR(INT2_vector)
{
	testing++;
	ICR1L=testing;
	 switch (Edge)
	 {
		 case SwICU_EdgeRisiging:
		 TCNT2=0;
		 timer2Start();
		 SwICU_SetCfgEdge( SwICU_EdgeFalling);// set to faling
		 Edge=SwICU_EdgeFalling;
		 break;
		 
		 case SwICU_EdgeFalling:	 
		 // SwICU_Read( &Eco_width);
		  Eco_width= TCNT2;
		  timer2Stop();
		  Edge=SwICU_EdgeRisiging;
		  SwICU_SetCfgEdge( SwICU_EdgeRisiging);
		 //SwICU_Stop();

		 break;

		 default:
		 break;
	 }
	 
}