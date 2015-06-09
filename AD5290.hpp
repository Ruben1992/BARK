#ifndef _AD5290_hpp		
#define _AD5290_hpp
class AD5290{
	public:
	void write(uint8_t data[], uint8_t length);
	void init();
};
extern AD5290 ad5290;

#endif
