
#include <stdint.h>
#include <avr/io.h>
#include "spi.hpp"
#include "AD5290.hpp"

#ifndef __AD5290_hpp_
    #define __AD5290_hpp_

void AD5290::write(uint8_t data[], uint8_t length){
    PORTD &= ~(1<<7);
    for (int i = 0; i < length; ++i)
    {
        spi.trans(data[i]);
    }

    PORTD |= (1<<7);
}
void AD5290::init(){
    DDRD |= (1<<7); // slave select
    spi.init();
}
AD5290 ad5290;
#endif