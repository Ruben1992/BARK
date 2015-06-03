#include <stdint.h>
#include <avr/io.h>
#include "USART.hpp"

#define F_CPU 16000000UL
#define BAUD 9600

#include <util/setbaud.h>

#ifndef F_CPU
    #define F_CPU 16000000UL // 16 MHz
#endif

void USART::init(uint32_t baud) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

USART::USART(){
}

uint8_t USART::receive(){
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}
 
void USART::send( uint8_t data){
 
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
 
}
 
void USART::send(uint8_t data[]){
 
	while(*data != 0x00){
		send(*data);
		data++;
		}
}

USART usart;