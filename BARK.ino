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


void buzz(int k){
    int i;
    int temp;
    temp = 1000/(2*k);

    for(int i=0; i<k*1000; i++){
        digitalWrite(5, HIGH); 
        delayMicroseconds(temp);
        digitalWrite(5, LOW);
        delayMicroseconds(temp);
    }

}

void setup(){
    pinMode(7, OUTPUT);
    pinMode(5, OUTPUT);
    buzz(10); 



    delay(1000);
    Serial.begin(115200);


    Serial.println("We just rebooted!!!!\n");
    // digitalWrite(7, LOW);
    // digitalWrite(7, HIGH);
    Serial.println("\t Start init");
    spi.init();
    Serial.println("\t end init");
    Serial.println("");
    // digitalWrite(7, LOW);
    // digitalWrite(7, HIGH);
    // digitalWrite(7, LOW);
    // digitalWrite(7, HIGH);
    delay(1);

}

void loop()
{
    //spi.init();

    // wiz.setIpData();
    //wiz.write(wiz.R_com, wiz.C_Mode, sock0.MR.TCP);
    Serial.println("\t begin setIpData");
    wiz.setIpData();
    Serial.println("\t end setIpData");
    delay(500);
    Serial.println("");
    Serial.println("\t sock init");
    Server sock0((uint16_t)0); // open socket 0, op 0
    Serial.println("end sock");
    while(1);
    //sock0.setPort(2000);
    // Serial.print("Status = ");
    // Serial.println(sock0.getStatus());

    // sock0.start();
    // Serial.print("Status = ");
    // Serial.println(sock0.getStatus());    

    // sock0.listen();
    // Serial.print("Status = ");
    // Serial.println(sock0.getStatus());  

}

