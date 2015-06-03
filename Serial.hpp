#ifndef _serial_hpp_
#define _serial_hpp_

class Serial {
private:
	/* data */
public:
	Serial();
	begin(int x);
	write(uint8_t data);
	write(uint8_t data[]);
};


#endif
