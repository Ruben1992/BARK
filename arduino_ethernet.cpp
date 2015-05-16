/*
 */
#include <stdint.h>
#define dummy 0x14
#define atmega328p
//#include <avr/io.h>
//#include <stdio.h>


#include "spi.cpp"
#include "ethernet.cpp"


/// Debug 1 = on, 0 = off
//#define debug  // decomment to disable debug information
#ifdef debug
  char buffer[100]; // SerialOut buffer;
#endif
  
void setup(){
  /// init SPI




/// Define IO
#ifdef debug
    Serial.begin(115200);
#endif
    // Insert code    
#ifdef debug
    Serial.println("Init");
#endif
    setIpData();
}
void loop()
{
    spi.init();
    wiz.setIpData();
    wiz.write(wiz.R_com, wiz.C_Mode, );


    while(1){
    ///while (SPI_write(wiz.r_com, wiz.c_Mode, 5) != -1);

    }

  
}






/* ------------------------------------------------------------------------------------- */
     
///// ************* RX-TX size ******************
     
//  SPI_write(wiz.R_com, wiz.C_TMSR, 0x55);
//#ifdef debug
//  Serial.print("\tsetting up Rx and Tx size to 2kB");
//#endif


//}

// void fill4bytes(uint8_t dest[], uint8_t data[]){
//   int i;
//   for (i = 0; i < 4; i++)
//     dest[i] = data[i];
  
// }

#ifdef debug
//  Serial.print("\tsetting up Rx and Tx size to 2kB");
//#endif


//}

// void fill4bytes(uint8_t dest[], uint8_t data[]){
//   int i;
//   for (i = 0; i < 4; i++)
//     dest[i] = data[i];
  
// }






