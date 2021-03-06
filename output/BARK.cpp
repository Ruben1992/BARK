/*
 */
/* *************************************************************************************************************************** */
//                                                includes        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */

#include <stdint.h>
#define __AVR_ATmega328p__
#include <avr/io.h>
#include <avr/interrupt.h>
/* *************************************************************************************************************************** */
//                                                settings        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */

#ifndef F_CPU
    #define F_CPU 16000000UL // 16 MHz
#endif
#define buffSize 100
// #define IPADRESS 10,0,0,222
#define IPADRESS 192,168,2,222

#define PORTNO 7010
#define daisyPots 4 /*amound of daisy chained pots*/
/* *************************************************************************************************************************** */
//                                                more includes        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */

#include <util/delay.h>     // mag later weggehaald worden, maar dit maakt sommige functies makkelijker voor mij
//#include <stdio.h>
#include "spi.hpp"
#include "ethernet.hpp"
#include <string.h> // Zie opmerking 1.
#include "USART.hpp"
#include "millis.hpp"
#include "AD5290.hpp"
#include "FlowSerial.h"

using namespace std;

/* *************************************************************************************************************************** */
//                                                PROTOTYPES        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */

void setup(); // prototype
void loop();  // prototype


/* *************************************************************************************************************************** */
//                                                GLOBALS        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */


/* *************************************************************************************************************************** */
//                                   main program (should not be touched, its holy)        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */

int main(){
    setup();
    loop();
    while(1); /// iets van een system reboot verzinnen?
    return 0;
}
/* *************************************************************************************************************************** */
//                                    initalization, should not be used for setting variables, since they      
//                                    will be destroyed on exit of this function.  
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */


void setup(){
    DDRD |= (1<<2);
    DDRD |= (1<<6);

    ad5290.init();
    spi.init();
    ip.setIp(IPADRESS);
    wiz.setIpData();
    usart.init((uint16_t)9);
}

/* *************************************************************************************************************************** */
//                                                temporary strings for usart.send (debugging)        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */
uint8_t error[] = {"error\r\n"}; 
uint8_t clDisC[] = {"Client disconnected\r\n"};
uint8_t clCon[] = {"Client connected\r\n"};
uint8_t Rsize[] = {"Ontvangen groote = "};
uint8_t Recv[] = {"Ontvangen: "};
uint8_t start[] = {"Server starting!!\n\r"};
uint8_t sockState[] = {"\t*** De server Status is "};
uint8_t sockEnd[] = {" ***\n\r"};


uint8_t hier[] = {"hier!!\n\r"};
uint8_t enter[] = {"\n\r"};

uint8_t name0[] = {"ERROR"};
uint8_t name1[] = {"SOCK_CLOSED"};
uint8_t name2[] = {"SOCK_INIT"};
uint8_t name3[] = {"SOCK_LISTEN"};
uint8_t name4[] = {"SOCK_ESTABLISHED"};
uint8_t name5[] = {"SOCK_CLOSE_WAIT"};
uint8_t name6[] = {"SOCK_IPRAW"};
uint8_t name7[] = {"SOCK_UDP"};
uint8_t name8[] = {"bestaat niet"};

/* *************************************************************************************************************************** */
//                                                the loop of our program        
/* *************************************************************************************************************************** */
/* *************************************************************************************************************************** */



void loop(){
    millis_init();
    uint8_t data[daisyPots]; // voor de digipots
    for(unsigned i = 0; i < daisyPots; data[i++] = 0); // alles op 0 zetten  
    uint32_t interval = millis;
    uint32_t speedLimit = millis;
    uint32_t aliveLedTim = millis;
    uint32_t digitPotSpeed = millis;

//     enum State{ERROR = 0, SOCK_CLOSED, SOCK_INIT, SOCK_LISTEN, SOCK_ESTABLISHED, SOCK_CLOSE_WAIT, SOCK_IPRAW, SOCK_UDP};
    uint8_t debugVal = 0;


    // State sock0.state = ERROR;
    // State lastsock0.state = ERROR;

    Server sock0((uint8_t)1); // open socket, nummer 0

    uint8_t inBuff[buffSize];
    uint16_t watchdog = 0;

    usart.send(start);
    uint16_t counter = 0;

    while(1){
        if (millis - speedLimit >= 0){  // assures there is a minimum of 1 ms delay;
            speedLimit = millis;
            
            switch(sock0.state){
            case Server::SOCK_CLOSED:  // connection closed, lets start the connection, eh?
                sock0.SockInterrupt(0xFF); //clear all interrupts

                sock0.state = Server::SOCK_INIT;
                /// clear all interupt registers
            break;
            case Server::SOCK_INIT:
                sock0.setPort((uint16_t)7010);
                if (sock0.start()){
                    if (sock0.listen()){
                        sock0.state = Server::SOCK_LISTEN;
                    }
                }
                else{
                    usart.send("FAILED!\n\r");
                    sock0.state = Server::SOCK_CLOSED;
                }
            break;
            case Server::SOCK_LISTEN:   // wait for incoming connection       
                // if interrupt from wiznet
                // 
                if (sock0.SockInterrupt() == 7){
                    sock0.state = Server::ERROR;
                }
                

                if(sock0.checkEstablished()){
                    usart.send(clCon);
                    sock0.state = Server::SOCK_ESTABLISHED;
                }
                /// if interupt = 007 then restart naar sock_closed
            break;
            case Server::SOCK_ESTABLISHED:
                static uint16_t last_received;
                uint16_t nowReceived;
                nowReceived = sock0.receivedData();
                // if (last_received != nowReceived){ /// overbodig
                //     last_received = nowReceived;
                // }
                // else{

                    if (nowReceived != 0){  /// hehe, eindelijk alles ontvangen, versturen maar
                        last_received++; // zorg er voor dat de voorwaarde van vorige meting niet nog een keer wordt getriggerd
                        
                        sock0.receivingData(inBuff, buffSize);

                        for (int i = 0; i < nowReceived; ++i)
                        {
                            flowSerial.update(inBuff[i]);
                            usart.write(inBuff[i]);
                            usart.send(" ");
                        }
                        usart.send("<- inbuff\r\n");
                        uint8_t outBuff[10+1];
                        uint8_t i;
                        for (i = 0;i < 10; ++i)
                        {
                            if (flowSerial.outboxAvailable() == 0){
                                outBuff[i] = 0;
                                break;
                            }
                            outBuff[i] = flowSerial.outboxNextOut();
                        }
                        usart.write(flowSerial.serialReg[0]);
                        usart.send(" ");
                        usart.write(flowSerial.serialReg[1]);
                        usart.send(" ");
                        usart.write(flowSerial.serialReg[2]);
                        usart.send(" ");
                        usart.write(flowSerial.serialReg[3]);
                        usart.send("\r\n");

                        // sock0.sendData(outBuff);
                        // usart.send("[byte 1 = ");
                        // usart.write(flowSerial.serialReg[1]);
                        // usart.send(" byte 2 = ");
                        // usart.write(flowSerial.serialReg[2]);
                        // usart.send(" byte 3 = ");
                        // usart.write(flowSerial.serialReg[3]);
                        // usart.send(" byte 4= ");
                        // usart.write(flowSerial.serialReg[4]);
                        // usart.send("]\n\r");


                        /*
                        usart.send(Rsize);
                        usart.write((uint8_t)nowReceived);
                        usart.send(enter);
                        if (sock0.receivingData(buff, buffSize)){
                            sock0.sendData(Recv);
                            sock0.sendData(buff);

                            usart.send(Recv);
                            usart.send(buff);
                        }
                        else{
                            sock0.sendData(error);
                            usart.send(error);
                        }

                        if (buff[0] == 's' && buff[1] == 't' && buff[2] == 'o' && buff[3] == 'p'){
                            sock0.disconnect();
                            sock0.state = Server::SOCK_CLOSED;
                        }
                        else
                            last_received++; // zorg er voor dat de voorwaarde van vorige meting niet nog een keer wordt getriggerd
                        */

                    }
//                }

                if(uint8_t temp = sock0.connectionDead()){
                    usart.send(clDisC);
                    usart.write(temp);
                    usart.send(enter);
                    sock0.state = Server::SOCK_CLOSED;
                }


                /*statements*/
            break;
            case Server::SOCK_CLOSE_WAIT:
            case Server::SOCK_IPRAW:
            case Server::SOCK_UDP:
            case Server::ERROR:
                usart.send("ERROR Detected!!\n\r");

                sock0.disconnect(); // close connection (CLEAN)
                sock0.close(); // close connection (DIRTY)
                sock0.state = Server::SOCK_CLOSED;
            break;
            default:
                sock0.state = Server::SOCK_CLOSED;
                //error
            }
        }



        if ((sock0.lastState != sock0.state) | (millis - interval >= 15000)){
            sock0.lastState = sock0.state;
            interval = millis;
            usart.send(sockState);
            usart.write(sock0.state);
            usart.send((uint8_t)' ');
            PORTD |= (1<<5);

            switch(sock0.state){
                case 0 :    usart.send(name0);  break;
                case 1 :    usart.send(name1);  break;
                case 2 :    usart.send(name2);  break;
                case 3 :    usart.send(name3);  break;
                case 4 :    usart.send(name4);  break;
                case 5 :    usart.send(name5);  break;
                case 6 :    usart.send(name6);  break;
                case 7 :    usart.send(name7);  break;
                default:    usart.send(name8);
            }
            usart.send("\tSR ");

            usart.write(sock0.getStatus());
            usart.send(" Glob ");
            usart.write(sock0.getInterrupt());
            usart.send(" sock ");
            usart.write(sock0.SockInterrupt());
            usart.send(sockEnd);
            PORTD &= ~(1<<5);
        }


        

        if(millis - digitPotSpeed >= 5){
            static uint8_t actualPosPots[daisyPots];
            digitPotSpeed = millis;

            uint8_t data[daisyPots];
            data[0] = (flowSerial.serialReg[0] * 21 ); // volume
            data[1] = 255-(flowSerial.serialReg[1] * 21 ); // treble
            data[2] = 255-(flowSerial.serialReg[2] * 21 ); // base
            data[3] = (flowSerial.serialReg[3]+3) * 21;

            for (int i = 0; i < daisyPots; ++i){ /// smoothly fades to desired value
                if (actualPosPots[i] < data[i])
                    actualPosPots[i]++;
                if (actualPosPots[i] > data[i])
                    actualPosPots[i]--;
            }
            ad5290.write(actualPosPots, (uint8_t)daisyPots);    
        }

        if(millis - aliveLedTim >= 500){
            aliveLedTim = millis;

            PORTD ^= (1<<2);

        }
       sock0.watchdog();

    }
}



