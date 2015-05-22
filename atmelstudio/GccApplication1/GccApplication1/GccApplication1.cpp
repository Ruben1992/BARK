/*
 */
#include <stdint.h>
#define dummy 0x14
#ifndef
	#define __AVR_ATmega328P__
#endif
#define atmega328p
#include <avr/io.h>
//#include <stdio.h>


#include "ethernet.h"
  
int main()
{
    spi.init();

    Server sock0(0);


    wiz.setIpData();
    //wiz.write(wiz.R_com, wiz.C_Mode, sock0.MR.TCP);


    while(1){
    ///while (SPI_write(wiz.r_com, wiz.c_Mode, 5) != -1);

    }
    return 0;
}







