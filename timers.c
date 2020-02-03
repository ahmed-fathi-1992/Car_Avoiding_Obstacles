/*
 * timers.c
 *
 * Created: 20/01/2020 16:19:42
 *  Author: ahmed_pc
 */ 



#include "timers.h"

#define  FCPU 16000000
#define  T0_CLOCK_MASK 0xF8
#define  T1_CLOCK_MASK 0xFFF8
#define  T2_CLOCK_MASK 0xF8

extern uint32_t spwm_duty_cycle;
extern EN_bits_t spwm_pin_1;
extern EN_bits_t spwm_pin_2;

//extern  spwm_duty_cycle;
//extern  spwm_pin;

static En_timer0Mode_t      TIMER0_MODE= T0_NORMAL_MODE;   
static En_timer1Mode_t      TIMER1_MODE= T1_NORMAL_MODE;
static En_timer2Mode_t      TIMER2_MODE= T2_NORMAL_MODE;

static En_timer0perscaler_t TIMER0_PRESCALER= T0_PRESCALER_64; 
static En_timer0perscaler_t TIMER1_PRESCALER= T1_PRESCALER_NO; 
static En_timer0perscaler_t TIMER2_PRESCALER= T2_PRESCALER_64; 

/*===========================Timer0 Control===============================*/
/**
 * Description:
 * @param control
 * @param initialValue
 * @param outputCompare
 * @param interruptMask
 */
void timer0Init(En_timer0Mode_t en_mode,En_timer0OC_t en_OC0,En_timer0perscaler_t en_prescal, uint8_t u8_initialValue, uint8_t u8_outputCompare, En_timer0Interrupt_t en_interruptMask)
{
	
	TCCR0= TCCR0 | en_mode;
	TCCR0= TCCR0 | en_OC0;
	TCCR0= TCCR0 | en_prescal;		
	
	TIMSK = TIMSK |  en_interruptMask;
	
	TCNT0 = u8_initialValue;
	
	OCR0 = u8_outputCompare;
	
	TIMER0_PRESCALER= en_prescal;
	
	TIMER0_MODE= en_mode; 
	
}

/**
 * Description:
 * @param value
 */
void timer0Set(uint8_t u8_value)
{
	TCNT0=u8_value;
}

/**
 * Description:
 * @return
 */
uint8_t timer0Read(void)
{
	return TCNT0;
		
}

/**
 * Description:
 */
void timer0Start(void)
{
	TCCR0 = TCCR0 | TIMER0_PRESCALER;
}

/**
 * Description:
 */
void timer0Stop(void)
{
	TCCR0 = T0_CLOCK_MASK & T0_NO_CLOCK;
}

/**
 * Description:
 * @param delay
 */
void timer0DelayMs(uint16_t u16_delay_in_ms)
{
	uint32_t index=0;
	uint32_t Number_of_tick_per_ms =0;
	
	Number_of_tick_per_ms= FCPU /(64000) ; //64 * 1000 // prescaler 64
	
	if(TIMER0_MODE == T0_NORMAL_MODE) //
	{
		timer0Start();
		for(index=0;index<u16_delay_in_ms;index++){
			timer0Set(256 - Number_of_tick_per_ms); // Preload with 6 counts if prescaler 64
			while(!(TIFR & (1 << 0)));
			TIFR |= (1 << 0);
		}
		timer0Stop();		
	}
	else
	{
		OCR0 = Number_of_tick_per_ms-1;
		timer0Start();
		for(index=0;index<u16_delay_in_ms;index++){
			while(!(TIFR & (1 << 1)));
			TIFR |= (1 << 1);
		}
		timer0Stop();		
		
	}





	
}

/*
 * user defined
 */
void timer0DelayUs(uint32_t u32_delay_in_us)
{
		
}

/**
 * Description:
 * @param dutyCycle
 */
void timer0SwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency,EN_bits_t swpwm_pin_1,EN_bits_t swpwm_pin_2)
{
	 spwm_duty_cycle= u8_dutyCycle;
	spwm_pin_1= swpwm_pin_1;
	spwm_pin_2= swpwm_pin_2;	
			
}









/*===========================Timer1 Control===============================*/
/**
 * Description:
 * @param controlA
 * @param controlB
 * @param initialValue
 * @param outputCompare
 * @param interruptMask
 */
void timer1Init(En_timer1Mode_t en_mode,En_timer1OC_t en_OC,En_timer1perscaler_t en_prescal, uint16_t u16_initialValue, uint16_t u16_outputCompareA, uint16_t u16_outputCompareB,uint16_t u16_inputCapture, En_timer1Interrupt_t en_interruptMask)
{
	TCCR1= TCCR1 | en_mode;
	TCCR1= TCCR1 | en_OC;
	TCCR1= TCCR1 | en_prescal;

	TIMSK = TIMSK |  en_interruptMask;
	
	TCNT1 = u16_initialValue;
	
	OCR1A = u16_outputCompareA; 
	OCR1B = u16_outputCompareB;
	
	TIMER1_PRESCALER= en_prescal;
	
	TIMER1_MODE= en_mode;
	
	
}

/**
 * Description:
 * @param value
 */
void timer1Set(uint16_t u16_value)
{
	TCNT1 =u16_value;	
}

/**
 * Description:
 * @return
 */
uint16_t timer1Read(void)
{
	return TCNT1 ; 
}

/**
 * Description:
 */
void timer1Start(void)
{
	TCCR1 = TCCR1 | TIMER1_PRESCALER;	
	
}

/**
 * Description:
 */
void timer1Stop(void)
{
	TCCR1B = T1_CLOCK_MASK & T1_NO_CLOCK; 

	
}

/**
 * Description:
 * @param delay
 */
void timer1DelayMs(uint16_t u16_delay_in_ms)
{
	uint32_t index=0;
	uint32_t Number_of_tick_per_ms =0;
	
	Number_of_tick_per_ms= FCPU /1000 ; //64 * 1000 // prescaler 64
	
	if(TIMER1_MODE == T1_NORMAL_MODE) //
	{
			timer1Start();
			for(index=0;index<u16_delay_in_ms;index++){
				timer1Set(65536 - Number_of_tick_per_ms ); 
				while(!(TIFR & (1 << 2)));
				TIFR |= (1 << 2);
			}
			timer1Stop();
	}else
	{
		  OCR1A = 249;
    		timer1Start();
		  for(index=0;index<u16_delay_in_ms;index++){
			while(!(TIFR & (1 << 4)));
			 TIFR |= (1 << 4);
			}
				timer1Stop();
	}
	
	
}

/*
 * user defined
 */
void timer1DelayUs(uint32_t u32_delay_in_us)
{
	
	
}

/**
 * Description:
 * @param dutyCycle
 */
void timer1SwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency)
{
	
	
}

/**
 * Description:
 * @param dutyCycle
 */
void timer1HwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency)
{
	
	
}




/*===========================Timer2 Control===============================*/
/**
 * Description:
 * @param control
 * @param initialValue
 * @param outputCompare
 * @param interruptMask
 */
void timer2Init(En_timer2Mode_t en_mode,En_timer2OC_t en_OC,En_timer2perscaler_t en_prescal, uint8_t u8_initialValue, uint8_t u8_outputCompare, uint8_t u8_assynchronous, En_timer2Interrupt_t en_interruptMask)
{
	TCCR2= TCCR2 | en_mode;
	TCCR2= TCCR2 | en_OC;
	TCCR2= TCCR2 | en_prescal;
	
	TIMSK = TIMSK |  en_interruptMask;
	
	TCNT2 = u8_initialValue;
	
	OCR2 = u8_outputCompare;
	
	TIMER2_PRESCALER= en_prescal;
	
	TIMER2_MODE= en_mode;	
	
	
}
/**
 * Description:
 * @param value
 */
void timer2Set(uint8_t u8_a_value)
{
	TCNT2 = u8_a_value;
	
}

/**
 * Description:
 * @return
 */
uint8_t timer2Read(void)
{
	return TCNT2 ;
	
}

/**
 * Description:
 */
void timer2Start(void)
{
//TCCR2 = TCCR2 | TIMER2_PRESCALER;
TCCR2 = TCCR2 | TIMER2_PRESCALER;	
	
}

/**
 * Description:
 */
void timer2Stop(void)
{
TCCR2 = TCCR2 & T2_CLOCK_MASK  ; 
}

/**
 * Description:
 * @param delay
 */
void timer2DelayMs(uint16_t u16_delay_in_ms)
{
	uint32_t index=0;
	uint32_t Number_of_tick_per_ms =0;
	
	Number_of_tick_per_ms= FCPU /(64000) ; //64 * 1000 // prescaler 64
	
	if(TIMER1_MODE == T1_NORMAL_MODE) //
	{
		timer2Start();
		for(index=0;index<u16_delay_in_ms;index++){
			timer1Set(256 - Number_of_tick_per_ms); // Preload with 6 counts if prescaler 64
			while(!(TIFR & (1 << 6)));
			TIFR |= (1 << 6);
		}
		timer2Stop();
	}
	else
	{
		OCR2 = Number_of_tick_per_ms-1;
		timer2Start();
		for(index=0;index<u16_delay_in_ms;index++){
			while(!(TIFR & (1 << 7)));
			TIFR |= (1 << 7);
		}
		timer2Stop();
		
	}
	
	
}

/*
 * user defined
 */
void timer2DelayUs(uint32_t u16_delay_in_us)
{
	
	
}

/**
 * Description:
 * @param dutyCycle
 */
void timer2SwPWM(uint8_t u8_dutyCycle,uint8_t u8_frequency)
{
	
	
	
}