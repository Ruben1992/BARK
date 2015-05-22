/*
 */
#include <stdint.h>
#define dummy 0x14
#define __AVR_ATmega328P__
#define atmega328p
#include <avr/io.h>
//#include <stdio.h>


#include "ethernet.cpp"


/// Debug 1 = on, 0 = off
//#define debug  // decomment to disable debug information
#ifdef debug
	char buffer[100]; // SerialOut buffer;
#endif

void setup(){
  /// init SPI




/// Define IO
#ifdef debug
    Serial.begin(115200);
#endif
    // Insert code    
#ifdef debug
    Serial.println("Init");
#endif
    // wiz.setIpData();
}
void loop()
{
	init();
    spi.init();

    Server sock0(0);


    // wiz.setIpData();
    //wiz.write(wiz.R_com, wiz.C_Mode, sock0.MR.TCP);


    while(1){
    ///while (SPI_write(wiz.r_com, wiz.c_Mode, 5) != -1);

    }
}







