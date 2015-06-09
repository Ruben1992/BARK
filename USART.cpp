#include <stdint.h>
#include <avr/io.h>
#include "USART.hpp"

#define F_CPU 16000000UL
#define BAUD 19200

//#define USE_2X 1

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

void USART::send(char data[]){
 
    while(*data != 0x00){
        send((uint8_t) *data);
        data++;
    }
}



void USART::write( uint8_t arr){
    uint8_t scratch[3];
    scratch[0] = 0;
    scratch[1] = 0;
    scratch[2] = 0;


    int j, k;
    uint8_t nscratch = 3;
    int smin = nscratch-2;    /* speed optimization */
 

    for (j=0; j < 16; ++j) {
        /* This bit will be shifted in on the right. */
        int shifted_in = (arr & (1 << (15-j)))? 1: 0;

        /* Add 3 everywhere that scratch[k] >= 5. */
        for (k=smin; k < nscratch; ++k)
          	scratch[k] += (scratch[k] >= 5)? 3: 0;

        /* Shift scratch to the left by one position. */
        if (scratch[smin] >= 8)
          	smin -= 1;
        for (k=smin; k < nscratch-1; ++k) {
            scratch[k] <<= 1;
            scratch[k] &= 0xF;
            scratch[k] |= (scratch[k+1] >= 8);
        }

        /* Shift in the new bit from arr. */
        scratch[nscratch-1] <<= 1;
        scratch[nscratch-1] &= 0xF;
        scratch[nscratch-1] |= shifted_in;
    }

 
	

    /* Convert the scratch space from BCD digits to ASCII. */
    for (k=0; k < nscratch; ++k){
    	send(scratch[k] += '0');	
    }





}
USART usart;