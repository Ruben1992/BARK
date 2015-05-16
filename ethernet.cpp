#ifndef __ethernet_cpp_
#define __ethernet_cpp_

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

    int8_t read(uint8_t regGroup, uint8_t reg, uint8_t data); /// return -1 on error
    int8_t write(uint8_t regGroup, uint8_t reg, uint8_t data); /// return -1 on error
    uint8_t setIpData();
    public:

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
    static const uint8_t Sx_MR       = 0x00;    /// MODE
    static const uint8_t Sx_CR       = 0x01;    /// COMMAND
    static const uint8_t Sx_IR       = 0x02;    /// Interrupt
    static const uint8_t Sx_SR       = 0x03;    /// Status

    static const uint8_t Sx_PORT0    = 0x04;    /// Source Port
    static const uint8_t Sx_PORT1    = 0x05;

    static const uint8_t Sx_DHAR0    = 0x06;    /// Destination Hardware address
    static const uint8_t Sx_DHAR1    = 0x07;
    static const uint8_t Sx_DHAR2    = 0x08;
    static const uint8_t Sx_DHAR3    = 0x09;
    static const uint8_t Sx_DHAR4    = 0x0A;
    static const uint8_t Sx_DHAR5    = 0x0B;

    static const uint8_t Sx_DIPR0    = 0x0C;    /// Destination IP Address
    static const uint8_t Sx_DIPR1    = 0x0D;
    static const uint8_t Sx_DIPR2    = 0x0E;
    static const uint8_t Sx_DIPR3    = 0x0F;

    static const uint8_t Sx_DPORT0   = 0x10;    /// Destination Port
    static const uint8_t Sx_DPORT1   = 0x11;
    static const uint8_t Sx_MSSR0    = 0x12;    /// Maximum segmet size
    static const uint8_t Sx_MSSR1    = 0x13;
    static const uint8_t Sx_PROTO    = 0x14;    /// Protocol in raw mode
    static const uint8_t Sx_TOS      = 0x15;    /// IP TOS
    static const uint8_t Sx_TTL      = 0x16;    /// IP TTL
    static const uint8_t Sx_TX_FSR0  = 0x20;    /// TX FRee size
    static const uint8_t Sx_TX_FSR1  = 0x21;
    static const uint8_t Sx_TX_RD0   = 0x22;    /// TX Read pointer
    static const uint8_t Sx_TX_RD1   = 0x23;
    static const uint8_t Sx_TX_WR0   = 0x24;    /// TX write Pointer
    static const uint8_t Sx_TX_WR1   = 0x25;
    static const uint8_t Sx_RX_RSR0  = 0x26;    /// RX Received size
    static const uint8_t Sx_RX_RSR1  = 0x27;
    static const uint8_t Sx_RX_RD0   = 0x28;    /// RX Read Pointer
    static const uint8_t Sx_RX_RD1   = 0x29;
    private:
    friend Network;

}wiz;



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
int8_t wiznet::write(uint8_t regGroup, uint8_t reg, uint8_t data){ /// return -1 on error
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

int8_t wiznet::read(uint8_t regGroup, uint8_t reg, uint8_t data){ /// return -1 on error
    uint8_t n = 0;
    n =  spi.trans(read);  // n = 0, when ok
    n += spi.trans(regGroup);     // n + 1 = 1, when ok 
    n += spi.trans(reg);          // n + 2 = 3, when ok
    n += spi.trans(14);         // n + 3 = 6, when ok
#ifdef debug
	Serial.print(n);
#endif
    return n; // should return 6 when all ok
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
#endif



class Server{
public:
    Server(uint8_t number);
    setPort();
    setPort(uint8_t laag);
    setPort(uint8_t hoog, uint8_t laag);
    setPort(uint16_t getal);

    void start();            // initalizatie
    void listen();                   // luisteren voor binnen koment verkeer
    void established();              // is er een verbinding tot stand gekomen?
    void receivedData();             // data ontvangen?
    void receivingData();            // er komt data binnen, nu verwerken
    void sendData();                 // verstuur data
    void gotFin();                   // fin: einde verbinding ontvangen?
    void closed();                   // verbinding is verbroken (denk ik)
    void timeout();                  // Timeout in de verbinding
    void close();                    // sluit de verbinding
private:
    const static uint8_t OPEN      0x01;
    const static uint8_t LISTEN    0x02;
    const static uint8_t CONNECT   0x04;
    const static uint8_t DISCON    0x08;
    const static uint8_t CLOSE     0x10;
    const static uint8_t SEND      0x20;
    const static uint8_t SEND_MAC  0x21;
    const static uint8_t SEND_KEEP 0x22;
    const static uint8_t RESV      0x40;

    uint8_t port[1];
    uint8_t status;
    uint8_t state;

    uint8_t sNr;                     // socket nummer

    uint16_t timeoutTimer; // verzin hier nog wat leuks mee
};

Server::Server(uint8_t number){
    if (number >= 0 && number =< 3)
        sNr = number +4;
    else
        number = 0;
}
Server::setPort():port{0,80}{ }
Server::setPort(uint8_t laag):port{0,laag}{ }
Server::setPort(uint8_t hoog, uint8_t laag):port{hoog,laag}{ }
Server::setPort(uint16_t getal){
    port[1] = getal;
    port[0] = (getal>>8);   // werkt dit?
}
          
          
void Server::start(){                               // 0x13 - initalizatie
    wiz.write(sNr, wiz.Sx_CR, 0x01);                //      Set Mode (to tcp)
    wiz.write(sNr, wiz.Sx_PORT0, port[0]);          //      Set port
    wiz.write(sNr, wiz.Sx_PORT1, port[1]);          //
    wiz.write(sNr, wiz.Sx_CR, OPEN); 
}
void Server::listen(){ }                            // 0x14 - luisteren voor binnen koment verkeer
void Server::established(){ }                       // 0x17 - is er een verbinding tot stand gekomen?
void Server::receivedData(){ }                      // data ontvangen?
void Server::receivingData(){ }                     // er komt data binnen, nu verwerken
void Server::sendData(){ }                          // verstuur data
void Server::gotFin(){ }                            // 0x18 - fin: einde verbinding ontvangen?
void Server::closed(){ }                            // 0x00-verbinding is verbroken (denk ik)
void Server::timeout(){ }                           // Timeout in de verbinding
void Server::close(){ }                             // sluit de verbinding