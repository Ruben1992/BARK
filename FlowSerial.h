
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
		char ID = 0;
        char serialReg[registerSize];

        //Functios:
        char update();
        char read();
        char available();
        //void sendDebugInfo(String text, int textLength);
        void write(char address, char out[], int quantity);
        void write(char address, char out);

	private:
		//FSM states
		const static char start					= 0x00;
		const static char startRecieved 		= 0x01;
		const static char instructionRecieved	= 0x02;
		const static char argumentsRecieved		= 0x03;
		const static char LSBchecksumRecieved	= 0x04;
		const static char MSBchecksumRecieved	= 0x05;
		const static char executeInstruction	= 0x06;

		//instruction codes
		const static char IDrequest 			= 0x00;
		const static char IDset      			= 0x01;
		const static char readRequest			= 0x02;
		const static char writeCommand			= 0x03;
		const static char dataReturn			= 0x04;
		const static char debugInfo				= 0x05;
		const static char debugStateCommand		= 0x06;
		const static char gotTransmissionError		= 0x07;

		//Declare register sizes
		char inboxBuffer[inboxBufferSize];
		char outboxBuffer[outboxBufferSize];
		char argumentBuffer[argumentBufferSize];

		//Variables
		char inboxBufferAt;
		char inboxAvailable;
		int checksumInbox;
		char outboxBufferAt;
		char outboxPending;
		char process;
		char instruction;

		char argumentBufferAt;

		char LSBchecksumIn;
		char MSBchecksumIn;

		//Functions to simplify the code
		void addToOutbox(char out);
        void transmissionError(bool resend);
        void outboxSend();
};

#endif //_flowSerial_h_
