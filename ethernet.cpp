#ifndef __ethernet_cpp_
#define __ethernet_cpp_
#define __AVR_ATmega328P__
#include <stdint.h>
#include <string.h>
#include "ethernet.hpp"
#include "spi.hpp"

#ifndef bufferSize
    #define bufferSize 2048
#endif
/// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define gSn_RX_MASK 0x07FF
#define gSn_TX_MASK 0x07FF


Network::Network(){
    ip[0]   = 10;   ip[1]   = 0;    ip[2]   = 0   ; ip[3]   = 123;  // default ip
    mac[0]  = 0x00; mac[1]  = 0x08; mac[2]  = 0xDC; mac[3]  = 0xDE; mac[4]  = 0xAF; mac[5]  = 0x01;
    gate[0] = 10;   gate[1] = 0;    gate[2] = 0;    gate[3] = 1;
    SNM[0]  = 0xFF; SNM[1]  = 0xFF; SNM[2]  = 0xFF; SNM[3]  = 0x00;
}
void Network::setIp(uint8_t a, uint8_t b, uint8_t c, uint8_t d){
    ip[0] = a;  ip[1] = b; ip[2] = c; ip[3] = d;
}
void Network::setIp(uint8_t d){
    ip[3] = d;
}
void Network::setMac(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f){
    mac[0] = a; mac[1] = b; mac[2] = c; mac[3] = d; mac[4] = e; mac[5] = f;
}
void Network::setMac(uint8_t f){
    mac[5] = f;
}
void Network::setGate(uint8_t a, uint8_t b, uint8_t c, uint8_t d){
    gate[0] = a; gate[1] = b; gate[2] = c; gate[3] = d;
}
void Network::setSNM(uint8_t a, uint8_t b, uint8_t c, uint8_t d){
    SNM[0] = a; SNM[1] = b; SNM[2] = c; SNM[3] = d;
}

//wiznet::write
uint8_t wiznet::write(uint8_t regGroup, uint8_t reg, uint8_t data){ /// return -1 on error
    uint8_t n = 0;
    n =  spi.trans(M_write);      // n = 0, when ok
    n += spi.trans(regGroup);     // n + 1 = 1, when ok
    n += spi.trans(reg);          // n + 2 = 3, when ok
    n += spi.trans(data);         // n + 3 = 6, when ok
    #ifdef debug
    Serial.print(n);
    #endif
    return n; // should return 6 when all ok
}

uint8_t wiznet::read(uint8_t regGroup, uint8_t reg){ /// return -1 on error
    spi.trans(M_read);
    spi.trans(regGroup);
    spi.trans(reg);
    return(spi.trans(14));         // you can also find the reply in SPDR register
}
void wiznet::readRx(uint8_t dest[], uint8_t sNr, uint16_t startAdress, uint16_t length){
    uint16_t i;
    for (i = 0; i < length; i++){
        dest[i] = read(sNr, startAdress+i);
    }
}
void wiznet::writeTx(uint8_t source[], uint8_t sNr, uint16_t startAdress, uint16_t length){
    uint16_t i;
    for (i = 0; i < length; i++){
        write(sNr, startAdress+i, source[i]); /// return -1 on error
    }
}



uint8_t wiznet::setIpData(){
    int i;
    write(wiz.R_com, wiz.C_Mode, 0x80);   // set mode
#ifdef debug
    Serial.println("\tWiznetmode = 0x80");
#endif
///// ************* Gateway-Address ******************
    for (i=0; i<4; i++) // seg default gate way
        write(wiz.R_com, wiz.C_GAR0  + i, ip.gate[i]);
#ifdef debug
    Serial.print("\tsetting gateway to \t");
    for (i= 0; i<4; Serial.print(ip.gate[i++])) Serial.print(".");
    Serial.println("");
#endif
///// ************* subnetmask-Address ******************
    for (i=0; i<4; i++)  // set Network mask
    	write(wiz.R_com, wiz.C_SUBR0 + i, ip.SNM[i]);
#ifdef debug
    Serial.print("\tsetting subnetmask to \t");
    for (i= 0; i<4; Serial.print(ip.SNM[i++])) Serial.print(".");
    Serial.println("");
#endif
///// ************* MAC-Address ******************
    for (i=0; i<6; i++)  // set mac address
    	write(wiz.R_com, wiz.C_SHAR0 + i, ip.mac[i]);
#ifdef debug
    Serial.print("\tsetting mac to \t\t");
    for (i= 0; i<6; Serial.print(ip.mac[i++])) Serial.print(".");
    Serial.println("");
#endif

///// ************* IP-Address ******************
    for (i= 0; i<4; i++)  // set ip address
        write(wiz.R_com, wiz.C_SIPR0 + i, ip.ip[i]);
    
#ifdef debug
    Serial.print("\tsetting Ip to \t\t");
    for (i= 0; i<4; Serial.print(ip.ip[i++])) Serial.print(".");
    Serial.println("");
#endif
}
//#endif



Server::Server(uint8_t Number){
    if (Number >= 0 && Number <= 3){
        sNr    = Number +4;
        number = Number;
    }
    else{
        sNr    = 99; /// ERROR
        number = 95;
    }
}
void Server::setPort(){                             port[0] = 0;    port[1] = 80;}
void Server::setPort(uint8_t laag){                 port[0] = 0;    port[1] = laag;}
void Server::setPort(uint8_t hoog, uint8_t laag){   port[0] = hoog; port[1] = laag;}
void Server::setPort(uint16_t getal){
    port[1] = getal;
    port[0] = (getal>>8);   // werkt dit?
}
void Server::setPort(int x){
    port[1] = x;
    port[0] = (x>>8);   // werkt dit?
}

int Server::start(){                               // 0x13 - initalizatie
    wiz.write(sNr, wiz.Sn_CR, MR.TCP);                  //      Set Mode (to tcp)
    wiz.write(sNr, wiz.Sn_PORT0, port[0]);              //      Set port
    wiz.write(sNr, wiz.Sn_PORT1, port[1]);              //
    wiz.write(sNr, wiz.Sn_CR, CR.OPEN);                 //      command- open
    /// delay er tussen misschien?
    if (wiz.read(sNr, wiz.Sn_CR) != SR.INIT){
        wiz.write(sNr, wiz.Sn_CR, CR.CLOSE);
        return 0;
    }
    return 1;
}
int Server::listen(){                              // 0x14 - luisteren voor binnen koment verkeer
    wiz.write(sNr, wiz.Sn_CR, CR.LISTEN);
    if (wiz.read(sNr, wiz.Sn_SR) != SR.LISTEN){
        wiz.write(sNr, wiz.Sn_CR, CR.CLOSE);
        return 0;
    }
    return 1;
}
int Server::checkEstablished(){                         // 0x17 - is er een verbinding tot stand gekomen?
    if (wiz.read(sNr, wiz.Sn_SR) != SR.ESTABLISHED){
        return 0;
    }
    return 1;
}
uint16_t Server::receivedData(){                          // data ontvangen?
    return read2byte(sNr, wiz.Sn_RX_RSR0, wiz.Sn_RX_RSR1);
}

void Server::receivingData(){                       // er komt data binnen, nu verwerken

    //get_size = Sn_RX_RSR; // get the received size
    uint16_t get_size = read2byte(sNr, wiz.Sn_RX_RSR0, wiz.Sn_RX_RSR1);

    //get_offset = Sn_RX_RD & gSn_RX_MASK;  // calculate the offset address
    // Sn_RX_RD -> read pointer register
    // gSn_RX_MASK -> 2k-1 = 2047 = 0x07FF   /// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    uint16_t get_offset = (read2byte(sNr, wiz.Sn_RX_RD0, wiz.Sn_RX_RD1) & gSn_RX_MASK);

    //get_start_address = gSn_RX_BASE + get_offset;  // calculate the start address (physical address)
    // gSn_RX_BASE -> ????  sNr ????

    uint16_t gSn_RX_BASE = 0x6000 + (gSn_RX_MASK + 1) * number;      /// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    uint16_t get_start_address = gSn_RX_BASE + get_offset;



/* if overflow socket Rx memory */
    if ((get_offset + get_size) > gSn_RX_MASK + 1){
        /* Copy upper_size bytes of get_start_address to destination addr */
        uint16_t upper_size = (gSn_RX_MASK + 1) - get_offset;
//    void readRx(uint8_t dest[], uint8_t sNr, uint16_t startAdress, uint16_t length);

        wiz.readRx(buffer, sNr, get_start_address, upper_size);

        //memcpy(get_start_address, buffer , upper_size);
        /*update destination_addr*/
        //destination_addr += upper_size;

        /* copy left_size bytes of gSn_RX_BASE to destination_addr */
        uint16_t left_size = get_size - upper_size;

        //memcpy(gSn_RX_BASE, buffer , get_size);
        wiz.readRx(&buffer[upper_size], sNr, gSn_RX_BASE, left_size);

    }
    else{
        /* copy get_size bytes of get_start_address to destination_addr */
        wiz.readRx(buffer, sNr, get_start_address, get_size);
        //memcpy(get_start_address, buffer, get_size);
    }

    /* increase Sn_RX_RD as length of get_size */
    uint16_t temp = read2byte(sNr, wiz.Sn_RX_RD0, wiz.Sn_RX_RD1) + get_size;
    write2byte(sNr, wiz.Sn_RX_RD0, wiz.Sn_RX_RD1, temp);

    /* set RECV command */
    wiz.write(sNr, wiz.Sn_CR, CR.RESV);
}



int Server::sendData(uint8_t data[], uint16_t length){                            // verstuur data


    uint16_t gSn_RX_BASE = 0x6000 + (gSn_RX_MASK + 1) * number;      /// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



    uint16_t freeSpace = read2byte(sNr, wiz.Sn_TX_FSR0, wiz.Sn_TX_FSR1);
    if (length > freeSpace){
        return 0;   /// not enough space to send data
    }

    //get_offset = Sn_TX_WD & gSn_TX_MASK;  // calculate the offset address
    // Sn_TX_WD -> write pointer register
    // gSn_TX_MASK -> 2k-1 = 2047 = 0x07FF   /// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    uint16_t get_offset = (read2byte(sNr, wiz.Sn_TX_RD0, wiz.Sn_TX_RD1) & gSn_TX_MASK);

    //get_start_address = gSn_RX_BASE + get_offset;  // calculate the start address (physical address)
    // gSn_RX_BASE -> ????  sNr ????

    uint16_t gSn_TX_BASE = 0x4000 + (gSn_TX_MASK + 1) * number;      /// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    uint16_t get_start_address = gSn_TX_BASE + get_offset;

    if ( (get_offset + length) > (gSn_TX_MASK + 1)){
        /* Copy upper_size bytes of source_addr to get_stat_address*/
        uint16_t upper_size = (gSn_TX_MASK + 1) - get_offset;
        wiz.writeTx(data, sNr, get_start_address, upper_size);
        /* update source addr* */
        //destination_addr += upper_size;

        /* copy left_size bytes of source_addr to gSn_TX_BASE */
        uint16_t left_size = length - upper_size;
        wiz.writeTx(&data[upper_size], sNr, gSn_TX_BASE, left_size);
    }
    else{
        /* copy send_size bytes of source_addr to get_start_address */
        wiz.writeTx(data, sNr, get_start_address, length);
    }
    /* Increase Sn_TX_WR as length of send_size */
    uint16_t temp = read2byte(sNr, wiz.Sn_TX_WR0, wiz.Sn_TX_WR1) + length;
    /* set SEND command */
    wiz.write(sNr, wiz.Sn_CR, CR.SEND);
}


void Server::gotFin(){                              // 0x18 - fin: einde verbinding ontvangen?
    if (wiz.read(sNr, wiz.Sn_IR) & IR.DISCON){          // if disconnect request is fount (FIN), then close the connection
        disconnect();
    }
}
void Server::closed(){                              // de verbinding is (netjes,toch?)verbroken vanaf de anderekant
    if (wiz.read(sNr, wiz.Sn_IR) & IR.DISCON){          // if disconnect request is fount (FIN), then close the connection
        disconnect();
    }
}
int Server::timeout(){                             // check voor timeout
    if (wiz.read(sNr, wiz.Sn_IR) & IR.TIMEOUT){
        close();
        return 1;
    }
    return 0;
}
void Server::close(){                               // sluit de vieze methode
    wiz.write(sNr, wiz.Sn_CR, CR.CLOSE);
}
void Server::disconnect(){                          // sluit de vebinding netjes af
    wiz.write(sNr, wiz.Sn_CR, CR.DISCON);
}
uint8_t Server::getStatus(){                           // get status van de socket
    return wiz.read(sNr, wiz.Sn_SR);
}

uint16_t Server::read2byte(uint8_t group, uint8_t high, uint8_t low){ // lees 2 bytes uit en plaats ze in een word
    uint16_t _high = (uint16_t) wiz.read(group, high);
    uint16_t _low  = (uint16_t) wiz.read(group, low);
    return ((_high<<8) + _low);
}
void Server::write2byte(uint8_t group, uint8_t regHigh, uint8_t regLow, uint16_t data){
    uint8_t dataLow  = data;
    uint8_t datahigh = (data>>8);

    wiz.write(group, regHigh, datahigh);
    wiz.write(group, regLow, dataLow);
}


wiznet wiz;
Network ip;

#endif
