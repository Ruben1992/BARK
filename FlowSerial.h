
#ifndef _flowSerial_h_
#define _flowSerial_h_

#define inboxBufferSize     5
#define outboxBufferSize    30
#define registerSize        10
#define argumentBufferSize  7

class FlowSerial{
	public:
        //Constructors:
        FlowSerial();

		//Variables:
		uint8_t ID = 0;
        uint8_t serialReg[registerSize];

        //Functios:
        uint8_t update();
        uint8_t read();
        uint8_t available();
        //void sendDebugInfo(String text, int textLength);
        void write(uint8_t address, uint8_t out[], int quantity);
        void write(uint8_t address, uint8_t out);

	private:
		//FSM states
		const static uint8_t start					= 0x00;
		const static uint8_t startRecieved 		= 0x01;
		const static uint8_t instructionRecieved	= 0x02;
		const static uint8_t argumentsRecieved		= 0x03;
		const static uint8_t LSBchecksumRecieved	= 0x04;
		const static uint8_t MSBchecksumRecieved	= 0x05;
		const static uint8_t executeInstruction	= 0x06;

		//instruction codes
		const static uint8_t IDrequest 			= 0x00;
		const static uint8_t IDset      			= 0x01;
		const static uint8_t readRequest			= 0x02;
		const static uint8_t writeCommand			= 0x03;
		const static uint8_t dataReturn			= 0x04;
		const static uint8_t debugInfo				= 0x05;
		const static uint8_t debugStateCommand		= 0x06;
		const static uint8_t gotTransmissionError		= 0x07;

		//Declare register sizes
		uint8_t inboxBuffer[inboxBufferSize];
		uint8_t outboxBuffer[outboxBufferSize];
		uint8_t argumentBuffer[argumentBufferSize];

		//Variables
		uint8_t inboxBufferAt;
		uint8_t inboxAvailable;
		int checksumInbox;
		uint8_t outboxBufferAt;
		uint8_t outboxPending;
		uint8_t process;
		uint8_t instruction;

		uint8_t argumentBufferAt;

		uint8_t LSBchecksumIn;
		uint8_t MSBchecksumIn;

		//Functions to simplify the code
		void addToOutbox(uint8_t out);
        void transmissionError(bool resend);
        void outboxSend();
};

#endif //_flowSerial_h_
