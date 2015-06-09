#ifndef _MILLIS_hpp
#define _MILLIS_hpp
#include <avr/interrupt.h>

extern volatile uint32_t millis;
// ISR(TIMER1_CAPT_vect); // volgens mij overbodig om te protoypen


void millis_init();

#endif


