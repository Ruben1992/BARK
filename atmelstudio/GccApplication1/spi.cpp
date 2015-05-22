#include <stdint.h>
#define tempTestMode 1


#ifndef __SPI_cpp_
    #define __SPI_cpp_

    #ifdef __AVR_ATmega328P__
        //#pragma GCC warning "You uc is supported!"

       // #pragma message we support the atmega328P!! 
    #else

        #error "please the define the used microcontroller like "#define __AVR_ATmega328P__""
        #error "if you did define it correctly, then we do not support your microcontroller, sorry :("
    #endif

#ifndef header
    class SPI{
    public:
        void init();
        uint8_t trans(uint8_t cData);
    private:
    }spi;
#endif

    #ifdef __AVR_ATmega328P__
        #define DDR_SPI DDRB 
        #define DD_MOSI 5  
        #define DD_SCK 3   
        #define DD_SS 2     
        #define PORT_SPI0 PORT


        void SPI::init(){ 
            #if tempTestMode
            PORT_SPI0 |= (1<<DD_SS); // slave select
            /* Set MOSI and SCK output, all others input */
            DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK|(1<<DD_SS));
            /* Enable SPI, Master, set clock rate fck/16 */
            SPCR = (1<<SPE)|(1<<MSTR);
            /* Double speed mode, want sneller is beter */
            SPSR |= (1<<SPI2X);  
            #endif
        }
        uint8_t SPI::trans(uint8_t cData){
            #if tempTestMode
            /* Start transmission */
            PORT_SPI0 &= ~(1<<DD_SS);
            SPDR = cData;
            /* Wait for transmission complete */
            while(!(SPSR & (1<<SPIF)));
            PORT_SPI0 |= (1<<DD_SS);
            return SPDR;
            
            #else 
                return 0;
            #endif
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