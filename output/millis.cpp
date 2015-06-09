#include <avr/io.h>
#include <avr/interrupt.h>
#include "millis.hpp"

/* *************************************************************************************************************************** */
//                                                millis implementation, uses timer1        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */
volatile uint32_t millis;
void millis_init(){
    TCCR0A = (1<<WGM01);        //Timer in CTC mode
    TCCR0B = ((1<<CS01)|(1<<CS00));    //1:64 prescaler
    OCR0A = 249;            //Value to have an compare at every 1ms
    TIMSK0 = (1<<OCIE0A);        //Enable timer interrupts
    sei();                //Enable global interrupts
}
 
ISR(TIMER0_COMPA_vect){
    millis++;    //Increase milis count by one millisecond
}




// /* *************************************************************************************************************************** */
// //                                                millis implementation, uses timer1        
// /* *************************************************************************************************************************** */
//  *************************************************************************************************************************** 

// void millis_init(){
//     TCCR0A = (1<<WGM01);        //Timer in CTC mode
//     TCCR0B = ((1<<CS01)|(1<<CS00));    //1:64 prescaler
//     OCR0A = 249;            //Value to have an compare at every 1ms
//     TIMSK0 = (1<<OCIE0A);        //Enable timer interrupts
//     sei();                //Enable global interrupts
// }
 
// ISR(TIMER0_COMPA_vect){
//     millis++;    //Increase milis count by one millisecond
// }

