#include <stdint.h>
#define tempTestMode 1


#ifndef __SPI_h_
    #define __SPI_h_

    #ifdef __AVR_ATmega328P__
        //#pragma GCC warning "You uc is supported!"

       // #pragma message we support the atmega328P!! 
    #else

        #error "please the define the used microcontroller like "#define __AVR_ATmega328P__""
        #error "if you did define it correctly, then we do not support your microcontroller, sorry :("
    #endif


    class SPI{
    public:
        void init();
        uint8_t trans(uint8_t cData);
    private:
    }spi;

    #ifdef __AVR_ATmega328P__
        #define DDR_SPI DDRB 
        #define DD_MOSI 5  
        #define DD_SCK 3   
        #define DD_SS 2
        #define PORT_SPI0 PORTB

    #endif

#endif