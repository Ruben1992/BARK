#define header

#ifndef __ethernet_cpp_
#define __ethernet_cpp_
#include <stdint.h>
#include "spi.cpp"
#include <string.h>





#define bufferSize 2048
/// ONLY WORKS IF WE USE 2K PER SOCKET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define gSn_RX_MASK 0x07FF
#define gSn_TX_MASK 0x07FF

#ifndef header
class Network{
public:
    void setIp(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
    void setIp(uint8_t d);
    void setMac(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f);
    void setMac(uint8_t f);
    void setGate(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
    void setSNM(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
    Network();
    uint8_t ip[4];
    uint8_t mac[6];
    uint8_t gate[4];
    uint8_t SNM[4];
private:

}ip;

class wiznet{
    public:
    uint8_t read(uint8_t regGroup, uint8_t reg); /// return -1 on error
    uint8_t write(uint8_t regGroup, uint8_t reg, uint8_t data); /// return -1 on error
    void writeTx(uint8_t source[], uint8_t sNr, uint16_t startAdress, uint16_t length);
    void readRx(uint8_t dest[], uint8_t sNr, uint16_t startAdress, uint16_t length);
    uint8_t setIpData();


///-- First byte to send (selecting read, or write mode)
    static const uint8_t M_write     = 0xF0;    /// Write Mode
    static const uint8_t M_read      = 0x0F;    /// Read Mode
///-- Second byte to send (selecting register group)
    static const uint8_t R_com       = 0x00;    /// Common Register
    static const uint8_t R_s0        = 0x04;    /// Socket 0
    static const uint8_t R_s1        = 0x05;    /// Socket 1
    static const uint8_t R_s2        = 0x06;    /// Socket 2
    static const uint8_t R_s3        = 0x07;    /// Socket 3
///-- Third byte to send (acces specific register in a group)
    /// Common registers ****************************************************************************************************
    /// Common registers ****************************************************************************************************
    /// Common registers ****************************************************************************************************
    static const uint8_t C_Mode      = 0x00;    /// MODE

    static const uint8_t C_GAR0      = 0x01;    /// Gateway address
    static const uint8_t C_GAR1      = 0x02;
    static const uint8_t C_GAR2      = 0x03;
    static const uint8_t C_GAR3      = 0x04;

    static const uint8_t C_SUBR0     = 0x05;    /// Subnet mask address
    static const uint8_t C_SUBR1     = 0x06;
    static const uint8_t C_SUBR2     = 0x07;
    static const uint8_t C_SUBR3     = 0x08;

    static const uint8_t C_SHAR0     = 0x09;    /// Source Hardware address
    static const uint8_t C_SHAR1     = 0x0A;
    static const uint8_t C_SHAR2     = 0x0B;
    static const uint8_t C_SHAR3     = 0x0C;
    static const uint8_t C_SHAR4     = 0x0D;
    static const uint8_t C_SHAR5     = 0x0E;

    static const uint8_t C_SIPR0     = 0x0F;    /// source IP address
    static const uint8_t C_SIPR1     = 0x10;
    static const uint8_t C_SIPR2     = 0x11;
    static const uint8_t C_SIPR3     = 0x12;

    static const uint8_t C_IR        = 0x15;    /// Interrupt
    static const uint8_t C_IRM       = 0x16;    /// Interrupt Mask

    static const uint8_t C_RTR0      = 0x17;    /// Retry time
    static const uint8_t C_RTR1      = 0x18;
    static const uint8_t C_RCR       = 0x19;    /// Retry count

    static const uint8_t C_RMSR      = 0x1A;    /// Rx Memory size
    static const uint8_t C_TMSR      = 0x1B;    /// Tx Memory size

    static const uint8_t C_PATR0     = 0x1C;    /// Authentication Type in PPPoE
    static const uint8_t C_PATR1     = 0x1D;

    static const uint8_t C_PTIMER    = 0x28;    /// PPP LCP Request Timer
    static const uint8_t C_PMAGIC    = 0x29;    /// PPP LCP Magic number

    static const uint8_t C_UIPR0     = 0x2A;    /// Unreachable IP address
    static const uint8_t C_UIPR1     = 0x2B;
    static const uint8_t C_UIPR2     = 0x2C;
    static const uint8_t C_UIPR3     = 0x2D;

    static const uint8_t C_UPORT0    = 0x2E;    /// Unreachable Port
    static const uint8_t C_UPORT1    = 0x2F;
    ///---Socket registers   ***********************************************************************************************
    ///---Socket registers   ***********************************************************************************************
    ///---Socket registers   ***********************************************************************************************
    static const uint8_t Sn_MR       = 0x00;    /// MODE
    static const uint8_t Sn_CR       = 0x01;    /// COMMAND
    static const uint8_t Sn_IR       = 0x02;    /// Interrupt
    static const uint8_t Sn_SR       = 0x03;    /// Status

    static const uint8_t Sn_PORT0    = 0x04;    /// Source Port
    static const uint8_t Sn_PORT1    = 0x05;

    static const uint8_t Sn_DHAR0    = 0x06;    /// Destination Hardware address
    static const uint8_t Sn_DHAR1    = 0x07;
    static const uint8_t Sn_DHAR2    = 0x08;
    static const uint8_t Sn_DHAR3    = 0x09;
    static const uint8_t Sn_DHAR4    = 0x0A;
    static const uint8_t Sn_DHAR5    = 0x0B;

    static const uint8_t Sn_DIPR0    = 0x0C;    /// Destination IP Address
    static const uint8_t Sn_DIPR1    = 0x0D;
    static const uint8_t Sn_DIPR2    = 0x0E;
    static const uint8_t Sn_DIPR3    = 0x0F;

    static const uint8_t Sn_DPORT0   = 0x10;    /// Destination Port
    static const uint8_t Sn_DPORT1   = 0x11;
    static const uint8_t Sn_MSSR0    = 0x12;    /// Maximum segmet size
    static const uint8_t Sn_MSSR1    = 0x13;
    static const uint8_t Sn_PROTO    = 0x14;    /// Protocol in raw mode
    static const uint8_t Sn_TOS      = 0x15;    /// IP TOS
    static const uint8_t Sn_TTL      = 0x16;    /// IP TTL
    static const uint8_t Sn_TX_FSR0  = 0x20;    /// TX FRee size
    static const uint8_t Sn_TX_FSR1  = 0x21;
    static const uint8_t Sn_TX_RD0   = 0x22;    /// TX Read pointer
    static const uint8_t Sn_TX_RD1   = 0x23;
    static const uint8_t Sn_TX_WR0   = 0x24;    /// TX write Pointer
    static const uint8_t Sn_TX_WR1   = 0x25;
    static const uint8_t Sn_RX_RSR0  = 0x26;    /// RX Received size
    static const uint8_t Sn_RX_RSR1  = 0x27;
    static const uint8_t Sn_RX_RD0   = 0x28;    /// RX Read Pointer
    static const uint8_t Sn_RX_RD1   = 0x29;
    private:
    friend Network;

}wiz;
#endif


Network::Network(){
    ip[0]   = 10;   ip[1]   = 0;    ip[2]   = 0   ; ip[3]   = 200;  // default ip
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
    n =  spi.trans(M_write);  // n = 0, when ok
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
    while(write(wiz.R_com, wiz.C_Mode, 0x80) == -1);   // set mode
#ifdef debug
    Serial.println("\tWiznetmode = 0x80");
#endif
///// ************* Gateway-Address ******************
    for (i=0; i<4; write(wiz.R_com, wiz.C_GAR0  + i, ip.gate[i])) // seg default gate way
      i++;
#ifdef debug
    Serial.print("\tsetting gateway to \t");
    for (i= 0; i<4; Serial.print(ip.gate[i++])) Serial.print(".");
    Serial.println("");
#endif
///// ************* subnetmask-Address ******************
    for (i=0; i<4; write(wiz.R_com, wiz.C_SUBR0 + i, ip.SNM[i]))  // set Network mask
    	i++;
#ifdef debug
    Serial.print("\tsetting subnetmask to \t");
    for (i= 0; i<4; Serial.print(ip.SNM[i++])) Serial.print(".");
    Serial.println("");
#endif
///// ************* MAC-Address ******************
    for (i=0; i<6; write(wiz.R_com, wiz.C_SHAR0 + i, ip.mac[i]))  // set mac address
    	i++;
#ifdef debug
    Serial.print("\tsetting mac to \t\t");
    for (i= 0; i<6; Serial.print(ip.mac[i++])) Serial.print(".");
    Serial.println("");
#endif

///// ************* IP-Address ******************
    for (i= 0; i<4; write(wiz.R_com, wiz.C_SIPR0 + i, ip.ip[i]))  // set ip address
    	i++;
#ifdef debug
    Serial.print("\tsetting Ip to \t\t");
    for (i= 0; i<4; Serial.print(ip.ip[i++])) Serial.print(".");
    Serial.println("");
#endif
}
//#endif

#ifndef header
class Server{
public:
    uint8_t buffer[bufferSize];
    Server(uint8_t number);
    void setPort();
    void setPort(uint8_t laag);
    void setPort(uint8_t hoog, uint8_t laag);
    void setPort(uint16_t getal);

    int start();            // initalizatie
    int listen();                    // luisteren voor binnen koment verkeer
    int checkEstablished();          // is er een verbinding tot stand gekomen?
    uint16_t receivedData();         // data ontvangen?
    void receivingData();            // er komt data binnen, nu verwerken
    int sendData(uint8_t data[], uint16_t length);                           // verstuur data
    void gotFin();                   // fin: einde verbinding ontvangen?
    void closed();                   // verbinding is verbroken (denk ik)
    int timeout();                  // Timeout in de verbinding
    void close();                    // sluit de verbinding
    uint8_t getStatus(); 
    void disconnect();                     // sluit de vebinding netjes af
private:

    struct MODE{
        const static uint8_t MULTI          = 1<<7;
        const static uint8_t ND_MC          = 1<<5;
        const static uint8_t P3             = 1<<3;
        const static uint8_t P2             = 1<<2;
        const static uint8_t P1             = 1<<1;
        const static uint8_t P0             = 1<<0;
        const static uint8_t CLOSED         = 0x00;
        const static uint8_t TCP            = 0x01;
        const static uint8_t UDP            = 0x02;
        const static uint8_t IPRAW          = 0x03;
        const static uint8_t MACRAW         = 0x04;
        const static uint8_t PPPoE          = 0x05;
    }MR;

    struct COMMAND{
        const static uint8_t OPEN           = 0x01;
        const static uint8_t LISTEN         = 0x02;
        const static uint8_t CONNECT        = 0x04;
        const static uint8_t DISCON         = 0x08;
        const static uint8_t CLOSE          = 0x10;
        const static uint8_t SEND           = 0x20;
        const static uint8_t SEND_MAC       = 0x21;
        const static uint8_t SEND_KEEP      = 0x22;
        const static uint8_t RESV           = 0x40;
    }CR;

    struct INTERRUPT{
        const static uint8_t SEND_OK        = 1<<4;
        const static uint8_t TIMEOUT        = 1<<3;
        const static uint8_t RECV           = 1<<2;
        const static uint8_t DISCON         = 1<<1;
        const static uint8_t CON            = 1<<0;
    }IR;

    struct STATUS{
        const static uint8_t CLOSED         = 0X00;
        const static uint8_t INIT           = 0X13;
        const static uint8_t LISTEN         = 0X14;
        const static uint8_t ESTABLISHED    = 0X17;
        const static uint8_t CLOSE_WAIT     = 0X1C;
        const static uint8_t UDP            = 0X22;
        const static uint8_t IPRAW          = 0X32;
        const static uint8_t MACRAW         = 0X42;
        const static uint8_t PPPOE          = 0X5F;

        const static uint8_t SYNSENT        = 0X15;
        const static uint8_t SYNRECV        = 0X16;
        const static uint8_t FIN_WAIT       = 0X18;
        const static uint8_t CLOSING        = 0X1A;
        const static uint8_t TIME_WAIT      = 0X1B;
        const static uint8_t LAST_ACK       = 0X1D;
        const static uint8_t ARP            = 0X11; /// kan ook op address 21, 31 zijn, (zie datasheet)
    }SR;

    uint8_t port[2];
    uint8_t status;
    uint8_t state;

    uint8_t sNr, number;                     // socket nummer
    uint8_t retrycounter;
    uint16_t timeoutTimer; // verzin hier nog wat leuks mee
    uint16_t read2byte(uint8_t group, uint8_t high, uint8_t low);
    void write2byte(uint8_t group, uint8_t regHigh, uint8_t regLow, uint16_t data);

};
#endif

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


#endif