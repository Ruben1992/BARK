

#ifndef __ethernet_hpp_
#define __ethernet_hpp_

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

};

extern Network ip;

class wiznet{
    public:
    wiznet();
    uint8_t read(uint8_t regGroup, uint8_t reg); /// return -1 on error
    uint8_t write(uint8_t regGroup, uint8_t reg, uint8_t data); /// return -1 on error
    void writeTx(uint8_t source[], /*uint8_t sNr,*/ uint16_t startAdress, uint16_t length);
    void readRx(uint8_t dest[], /*uint8_t sNr,*/ uint16_t startAdress, uint16_t length);
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

};

extern wiznet wiz;

class Server{
public:
    enum State{ERROR = 0, SOCK_CLOSED, SOCK_INIT, SOCK_LISTEN, SOCK_ESTABLISHED, SOCK_CLOSE_WAIT, SOCK_IPRAW, SOCK_UDP};

    State state;
    State lastState;
    uint32_t lastUpdate;
    uint16_t maxTime;

    Server(uint8_t x);

    void setPort();
    void setPort(uint8_t laag);
    void setPort(uint8_t hoog, uint8_t laag);
    void setPort(uint16_t getal);

    //void setDestination(uint8_t a, uint8_t b, uint8_t c, uint8_t d);


    int         start();               // Initialize connection
    int         listen();              // Start listning on a the specified port
    int         receivingData(uint8_t buffer[], uint16_t maxSize);       // reads the RX buffer on the wiznet
    int         sendData(uint8_t data[]/*, uint16_t length*/);            // send data to the TX buffer

    int         checkEstablished();    // checks if the connection is established
    uint16_t    receivedData();        // checks how many byte of data we received and returns that value
    int         gotFin();              // checks if we received a fin (finshed) bit
    int         closed();              // checks if the connections is closed from the other side and if it is, it will call the disconnect function
    int         timeout();             // checks if the connection timedout
    int         connectionDead();      // checks the 3 above functions, returns 0 if connection is dead, otherwise 1


    void        close();               // close the connection (dirty)
    void        disconnect();          // close the connection (clean)
    uint8_t     getStatus();           // reads the status register of the socket
    uint8_t     getInterrupt();         // get inerrupt register status
    uint8_t     getSockInterrupt();

    void        watchdogSet(uint16_t x);
    int         watchdog();

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
        const static uint8_t RECV           = 0x40;
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

    uint8_t sNr, number;                     // socket nummer
    uint8_t retrycounter;
    uint16_t timeoutTimer; // verzin hier nog wat leuks mee
    uint16_t read2byte(uint8_t group, uint8_t high, uint8_t low);
    void write2byte(uint8_t group, uint8_t regHigh, uint8_t regLow, uint16_t data);

};

#endif

