#include <stdint.h>
#include <avr/io.h>

#ifndef __SPI_hpp_
	#define __SPI_hpp_

    class SPI{
    public:
        void init();
        uint8_t trans(uint8_t cData);
    private:
    };

extern SPI spi;
#endif
