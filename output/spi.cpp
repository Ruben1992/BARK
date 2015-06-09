#include <stdint.h>
#include <avr/io.h>

#ifndef __SPI_cpp_
    #define __SPI_cpp_

#include "spi.hpp"
    #ifdef __AVR_ATmega328P__
        // uc supported :D
    #elif defined __AVR_ATmega1280__
        
    #else
        #error "please the define the used microcontroller like "#define __AVR_ATmega328P__""
        #error "if you did define it correctly, then we do not support your microcontroller, sorry :("
    #endif

    #ifdef __AVR_ATmega328P__

        #define DDR_SPI DDRB 
        #define DD_MOSI 5  
        #define DD_SCK 3   
        #define DD_SS 2     
        #define PORT_SPI0 PORTB
    #endif
    #ifdef __AVR_ATmega1280__

        #define DDR_SPI DDRB 
        #define DD_MOSI 2  
        #define DD_SCK 1   
        #define DD_SS 4     
        #define extraSS 0
        #define PORT_SPI0 PORTB
    #endif

    void SPI::init(){ 
        /* Set MOSI and SCK output, all others input */
        DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);
        #ifdef extraSS
            DDR_SPI |= (1<<extraSS));
        #endif
        
        /* Enable SPI, Master, set clock rate fck/16 */
        SPCR = (1<<SPE)|(1<<MSTR);
        /* Double speed mode, want sneller is beter */
        SPSR |= (1<<SPI2X);  /// ff uit gezet, voor de logic analyzer
    }

    uint8_t SPI::trans(uint8_t cData){
        /* Start transmission */
        /*PORT_SPI0 &= ~(1<<DD_SS);*/
        SPDR = cData;
        /* Wait for transmission complete */
        while(!(SPSR & (1<<SPIF)));
        /*PORT_SPI0 |= (1<<DD_SS);*/
        return SPDR;
    }

    SPI spi;    




#endif
