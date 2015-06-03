
#ifndef __USART_hpp_
    #define __USART_hpp_ 
class USART
{
public:
	USART();
	void init(uint32_t baud);
	uint8_t receive();
	void send( uint8_t data);
	void send( uint8_t data[]);
};

extern USART usart;
#endif

