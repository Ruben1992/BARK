/*
 */

#include <stdint.h>
#define dummy 0x14
//#define __AVR_ATmega1280__
#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
    #define F_CPU 16000000UL // 16 MHz
#endif
#include <util/delay.h>     // mag later weggehaald worden, maar dit maakt sommige functies makkelijker voor mij
//#include <stdio.h>

#include "spi.hpp"
#include "ethernet.hpp"
#include <string.h> // Zie opmerking 1.
#include "USART.hpp"

using namespace std;

void setup(); // prototype
void loop();  // prototype

#define bufferSize 200;

int main(){
    setup();
    loop();
    while(1); /// iets van een system reboot verzinnen?
}
uint8_t sizeOf(uint8_t x[]); // prototype


void debug(uint8_t pos,uint8_t x){
    for (int i = 0; i < 8; ++i)
    {
        uint8_t temp;
        temp = x&(1<<i);

        if (temp){
            PORTD |= (1<<(pos*2));
        }
        else{
            PORTD &= ~(1<<(pos*2));
        }
        PORTD |= (1<<(pos*2+1));
        PORTD &= ~(1<<(pos*2+1));
    }
}


void debug2(uint8_t pos,uint8_t x,uint8_t y){
    for (int i = 0; i < 8; ++i)
    {
        uint8_t temp, temp2;
        temp = x&(1<<i);
        temp2 = y&(1<<i);
        if (temp){
            PORTD |= (1<<(pos));
        }
        else{
            PORTD &= ~(1<<(pos));
        }

        if (temp2){
            PORTD |= (1<<(pos+1));
        }
        else{
            PORTD &= ~(1<<(pos+1));
        }
        PORTD |= (1<<(pos+2));
        PORTD &= ~(1<<(pos+2));
    }
    PORTD &= ~(1<<pos+1<<(pos+1));

}


void setup(){
    spi.init();
    ip.setIp(10,0,0,222);
    wiz.setIpData();
    uint32_t temp = 9600;
    usart.init(temp);
}

void loop(){

    enum State{ERROR = 0, SOCK_CLOSED, SOCK_INIT,SOCK_LISTEN, SOCK_ESTABLISHED, SOCK_CLOSE_WAIT, SOCK_IPRAW, SOCK_UDP};
    uint8_t debugVal = 0;


    State state0 = ERROR;
    Server sock0((uint8_t)1); // open socket, nummer 0
    State state1 = ERROR;
    Server sock1((uint8_t)1); // open socket, nummer 0


    uint8_t bericht[] = {"HEHE, HIJ DOET HET!\n"};
    uint8_t bericht2[] = {"socket 2\n"};

    #define buffSize 100
    uint8_t buff[buffSize];
    while(1){
        switch(state0){
        case SOCK_CLOSED:  // connection closed, lets start the connection, eh?
            state0 = SOCK_INIT;
        break;
        case SOCK_INIT:
            sock0.setPort((uint16_t)2000);
            if (sock0.start()){
                if (sock0.listen()){
                    state0 = SOCK_LISTEN;
                }
            }
            else 
                state0 = SOCK_CLOSED;
        break;
        case SOCK_LISTEN:   // wait for incoming connection          
            // if interrupt from wiznet
            if(sock0.checkEstablished())
                state0 = SOCK_ESTABLISHED;
        break;
        case SOCK_ESTABLISHED:
            static uint16_t last_received;
            uint16_t nowReceived;
            nowReceived = sock0.receivedData();
            if (last_received != nowReceived){
                last_received = nowReceived;
            }
            else{
                if (last_received != 0){  /// hehe, eindelijk alles ontvangen, versturen maar


                    uint8_t temp[] = {"Ontvangen: "};
                    sock0.receivingData(buff, buffSize);
                    sock0.sendData(temp/*, sizeOf(temp)*/);
                    sock0.sendData(buff/*, sizeOf(buff)*/);

                    usart.send(temp);
                    usart.send(buff);


                    if (buff[0] == 's' && buff[1] == 't' && buff[2] == 'o' && buff[3] == 'p'){
                        sock0.disconnect();
                        state0 = SOCK_CLOSED;
                    }
                    
                    else
                        last_received++; // zorg er voor dat de voorwaarde van vorige meting niet nog een keer wordt getriggerd
                }
            }
            /*statements*/
        break;
        case SOCK_CLOSE_WAIT:
        case SOCK_IPRAW:
        case SOCK_UDP:
        case ERROR:
            state0 = SOCK_CLOSED;        
        default:
            state0 = SOCK_CLOSED;
            //error
        }

        // switch(state1){
        // case SOCK_CLOSED:  // connection closed, lets start the connection, eh?
        //     state1 = SOCK_INIT;
        // break;
        // case SOCK_INIT:
        //     sock1.setPort((uint16_t)2001);
        //     if (sock1.start()){
        //         if (sock1.listen()){
        //             state0 = SOCK_LISTEN;
        //         }
        //     }
        //     else 
        //         state1 = SOCK_CLOSED;
        // break;
        // case SOCK_LISTEN:   // wait for incoming connection          
        //     // if interrupt from wiznet
        //     if(sock1.checkEstablished())
        //         state1 = SOCK_ESTABLISHED;
        // break;
        // case SOCK_ESTABLISHED:
        //     static uint16_t last_received;
        //     uint16_t nowReceived;
        //     nowReceived = sock1.receivedData();
        //     if (last_received != nowReceived){
        //         last_received = nowReceived;
        //     }
        //     else{
        //         if (last_received != 0){  /// hehe, eindelijk alles ontvangen, versturen maar
        //             sock1.receivingData(buff, buffSize);
        //             sock1.sendData(bericht, sizeOf(bericht2));
        //             sock1.sendData(buff, sizeOf(buff));
        //             sock1.disconnect();
        //             state1 = SOCK_CLOSED;
        //         }
        //     }
        //     /*statements*/
        // break;
        // case SOCK_CLOSE_WAIT:
        // case SOCK_IPRAW:
        // case SOCK_UDP:
        // case ERROR:
        //     state1 = SOCK_CLOSED;        
        // default:
        //     state0 = SOCK_CLOSED;
        //     //error
        // }


      //  __builtin_avr_delay_cycles(16*1000*500);
       // debug2(0, state0, state1);
        _delay_ms(1);
    }
}




uint8_t sizeOf(uint8_t x[]){
    int i = -1;
    while(x[++i]);
    return i;
}




