#ifndef __STDINT_H_
#include <stdint.h>
#endif

#ifndef __SPI_cpp_
    #define __SPI_cpp_

    #ifdef atmega328p
        //#pragma GCC warning "You uc is supported!"

       // #pragma message we support the atmega328P!! 
    #else

        #error "please the define the used microcontroller like "#define atmega328p""
        #error "if you did define it correctly, then we do not support your microcontroller, sorry :("
    #endif


    class SPI{
    public:
        void init();
        uint8_t trans(uint8_t cData);
    private:
    }spi;

    #ifdef atmega328p
        #define DDR_SPI DDRB 
        #define DD_MOSI 5  
        #define DD_SCK 3   
        #define DD_SS 2
        #define PORT_SPI0 PORTB

        void SPI::init(){ 
            PORT_SPI0 |= (1<<DD_SS); // slave select
            /* Set MOSI and SCK output, all others input */
            DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK|(1<<DD_SS));
            /* Enable SPI, Master, set clock rate fck/16 */
            SPCR = (1<<SPE)|(1<<MSTR);
            /* Double speed mode, want sneller is beter */
            SPSR |= (1<<SPI2X);  
        }
        uint8_t SPI::trans(uint8_t cData){
            /* Start transmission */
            PORT_SPI0 &= ~(1<<DD_SS);
            SPDR = cData;
            /* Wait for transmission complete */
            while(!(SPSR & (1<<SPIF)));
            PORT_SPI0 |= (1<<DD_SS);
            return SPDR;
        }
    #endif
    // void SPI_init_master (void)
    // {
      
    //     PORT_SPI0 |= (1<<DD_SS); // slave select
    //     /* Set MOSI and SCK output, all others input */
    //     DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK|(1<<DD_SS));
    //     /* Enable SPI, Master, set clock rate fck/16 */
    //     SPCR = (1<<SPE)|(1<<MSTR);
    //     /* Double speed mode, want sneller is beter */
    //     SPSR |= (1<<SPI2X); 
    // }

    // uint8_t SPI_trans(uint8_t cData)
    // {
    //     /* Start transmission */
    //     PORT_SPI0 &= ~(1<<DD_SS);
    //     SPDR = cData;
    //     /*Wait for transmission complete */
    //     while(!(SPSR & (1<<SPIF)));
    //     PORT_SPI0 |= (1<<DD_SS);
    //     return SPDR;
    // }
#endif