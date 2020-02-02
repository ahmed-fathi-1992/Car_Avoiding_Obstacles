/*
 * interrupt.h
 *
 * Created: 23/01/2020 19:19:42
 *  Author: ahmed_pc
 */

#ifndef INTERRUPT_H_
#define INTERRUPT_H_


#define INT0_vector			   __vector_1
#define INT1_vector			   __vector_2
#define INT2_vector			   __vector_3
#define TIMER2_COMP_vector	   __vector_4
#define TIMER2_OVF_vector      __vector_5
#define TIMER1_CAPT_vector     __vector_6
#define TIMER1_COMPA_vector	   __vector_7
#define TIMER1_COMPB_vector    __vector_8
#define TIMER1_OVF_vector      __vector_9
#define TIMER0_COMP_vector     __vector_10
#define TIMER0_OVF_vector      __vector_11
#define SPI_STC_vector         __vector_12
#define USART_RXC_vector       __vector_13
#define USART_UDRE_vector      __vector_14
#define USART_TXC_vector       __vector_15
#define ADC_vector             __vector_16
#define EE_RDY_vector          __vector_17
#define ANA_COMP_vector        __vector_18
#define TWI_vector             __vector_19
#define SPM_RDY_vector         __vector_20

#define ISR(vector, ...)           \
   void vector (void) __attribute__ ((signal, used, externally_visible)) __VA_ARGS__ ; \
   void vector (void)

#define sei()  __asm__ __volatile__ ("sei" ::)
#define cli()  __asm__ __volatile__ ("cli" ::)

#endif /* INTERRUPT_H_ */
