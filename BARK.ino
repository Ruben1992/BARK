/*
 */

#include <stdint.h>
#define dummy 0x14
#define __AVR_ATmega328P__
#define atmega328p
#include <avr/io.h>
//#include <stdio.h>

#include "spi.hpp"
#include "ethernet.hpp"


void setup(){
    delay(500); // ff opstart tijd geven
    spi.init();
    Serial.begin(115200);
    delay(1);
}

void loop()
{
    //spi.init();

    // wiz.setIpData();
    //wiz.write(wiz.R_com, wiz.C_Mode, sock0.MR.TCP);
    wiz.setIpData();
    delay(500);
    //Server sock0(0); // open socket 0, op 0


    // sock0.setPort(2000);
    // Serial.print("Status = ");
    // Serial.println(sock0.getStatus());

    // sock0.start();
    // Serial.print("Status = ");
    // Serial.println(sock0.getStatus());    

    // sock0.listen();
    // Serial.print("Status = ");
    // Serial.println(sock0.getStatus());  

}

