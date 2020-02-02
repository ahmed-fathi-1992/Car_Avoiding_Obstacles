/*
 * SwICU.c
 *
 * Created: 29/01/2020 14:09:18
 *  Author: ahmed_pc
 */ 
 
#include "SwICU.h"
#include "registers.h"

extern volatile EN_SwICU_Edge_t Edge; //used in interrupt to save last interrupt state   
extern volatile uint8_t  Eco_width; //used in interrupt to save sensor puls width  to be converted in distance later 

void SwICU_Init(EN_SwICU_Edge_t a_en_inputCaptureEdge)
{
	timer2Init(T2_NORMAL_MODE,T2_OC2_DIS,T2_PRESCALER_256, 0, 0, 0,T2_POLLING);

	gpioPinDirection(SwICU_GPIO, SwICU_pin, INPUT); //configure the external interrupt PIN T2 to be input
	
SwICU_SetCfgEdge(SwICU_EdgeRisiging);	
	
}

/*
EN_SwICU_Edge_t SwICU_GetCfgEdge(void)
{
	
	
}
*/


void SwICU_SetCfgEdge(EN_SwICU_Edge_t a_en_inputCaptureEdgeedge)
{
  switch (a_en_inputCaptureEdgeedge)
  {
	 case SwICU_EdgeRisiging:
	 MCUCSR_ = MCUCSR_ |INT2_Edge_MASK;    //configure the external interrupt T2 to be at Risiging edge
	 break;
	 
	 case SwICU_EdgeFalling:
	 MCUCSR_ = MCUCSR_ & (~INT2_Edge_MASK);   //configure the external interrupt T2 to be at falling edge
	 break;

	 default:
	 break;
  }	
	
}
void SwICU_Read(volatile uint8_t * a_pu8_capt)
{
	*a_pu8_capt= TCNT2;	
}

void SwICU_Stop(void)
{
	GICR_ = GICR_ & (~INT2_En_MASK); //stop interrupt 	
}

void SwICU_Start(void)
{
    Edge=SwICU_EdgeRisiging;
	Eco_width=0;
	GICR_ = GICR_ | INT2_En_MASK;	//start interrupt
}

/*
void SwICU_Enable(void)
{
	
	
}
void SwICU_Disable(void)
{
	
	
}
*/