#include <stdint.h>
#include "FlowSerial.h"
#include "ethernet.hpp"

#define debugEnable 0

FlowSerial::FlowSerial():ID(0){}

uint8_t FlowSerial::update(uint8_t byteIn){

    switch(process){
        case start:
            //Check if the start byte equals 0xAA. Else stay in idle
            if(byteIn == 0xAA){
                checksumInbox = 0xAA;
                //timeoutTime = millis();
                process = startRecieved;
                #if debugEnable
                    sendDebugInfo("0xAA recieved.", 14);
                #endif // debugEnable
            }
            break;
        case startRecieved:
            instruction = byteIn;
            checksumInbox += byteIn;
            argumentBufferAt = 0;
            process = instructionRecieved;
            #if debugEnable
                Serial.println("instruction recieved");
            #endif // debugEnable
            break;
        case instructionRecieved:
            switch(instruction){
                case IDrequest:
                    LSBchecksumIn = byteIn;
                    process = LSBchecksumRecieved;
                    #if debugEnable
                        Serial.println("LSB recieved");
                    #endif // debugEnable
                    break;
                case readRequest:
                    argumentBuffer[argumentBufferAt++] = byteIn;
                    if(argumentBufferAt >= 2){
                        process = argumentsRecieved;
                    }
                    checksumInbox += byteIn;
                    break;
                case writeCommand:
                    argumentBuffer[argumentBufferAt++] = byteIn;
                    if(argumentBufferAt >= 2 && argumentBufferAt >= argumentBuffer[1] + 2){
                        process = argumentsRecieved;
                    }
                    checksumInbox += byteIn;
                    break;
                case dataReturn:
                    argumentBuffer[argumentBufferAt++] = byteIn;
                    if(argumentBufferAt >= 1 && argumentBufferAt >= argumentBuffer[0] + 2){
                        process = argumentsRecieved;
                    }
                    checksumInbox += byteIn;
                    break;
            }
            break;
        case argumentsRecieved:
            LSBchecksumIn = byteIn;
            #if debugEnable
                Serial.println("LSB recieved");
            #endif // debugEnable
            process = LSBchecksumRecieved;
            break;
        case LSBchecksumRecieved:
            MSBchecksumIn = byteIn;
            #if debugEnable
                Serial.println("MSB recieved");
            #endif // debugEnable
            process = MSBchecksumRecieved;
        case MSBchecksumRecieved:
            if((MSBchecksumIn << 8) | LSBchecksumIn == checksumInbox){
                #if debugEnable
                    Serial.println("checksum ok. exucute instruction");
                #endif // debugEnable
            }
            else{
                process = start;
                instruction = gotTransmissionError;
                #if debugEnable
                    Serial.print("checksum faulty. ");
                    Serial.print((MSBchecksumIn << 8) | LSBchecksumIn);
                    Serial.print("recieved. needed: ");
                    Serial.println(checksumInbox);
                #endif // debugEnable
                return -1;
            }
            //execute corresponding command
            switch(instruction){
                    case IDrequest:
                    {
                        //execude ID request
                        addToOutbox(0xAA);
                        addToOutbox(0x03);
                        addToOutbox(0x01);
                        addToOutbox(ID);
                        uint16_t serialSum = 0xAE + ID;
                        addToOutbox(serialSum & 0x00FF);
                        addToOutbox(serialSum >> 8);

                        //reset the state machine
                        process = start;
                        break;
                    }
                    case IDset:
                        ID = argumentBuffer[0];
                        process = start;
                        break;
                    case readRequest:
                    {
                        //execude read request
                        addToOutbox(0xAA);
                        addToOutbox(0x03);
                        addToOutbox(argumentBuffer[1]);
                        uint16_t checksumOut = 0xAD + argumentBuffer[1];
                        for(uint16_t i = 0;i < argumentBuffer[1]; i++){
                            addToOutbox(serialReg[i + argumentBuffer[0]]);
                            checksumOut += serialReg[i + argumentBuffer[0]];
                        }
                        addToOutbox(uint8_t(checksumOut));
                        addToOutbox(uint8_t(checksumOut >> 8));

                        //Reset the state machine
                        process = start;
                        break;
                    }
                    case writeCommand:
                    {
                        //Execude write command
                        for(uint16_t i = 0;i < argumentBuffer[1]; i++){
                            serialReg[i + argumentBuffer[0]] = argumentBuffer[i + 2];
                        }
                        //execude read request
                        addToOutbox(0xAA);
                        addToOutbox(0x03);
                        addToOutbox(argumentBuffer[1]);
                        uint16_t checksumOut = 0xAD + argumentBuffer[1];
                        for(uint16_t i = 0;i < argumentBuffer[1]; i++){
                            addToOutbox(serialReg[i + argumentBuffer[0]]);
                            checksumOut += serialReg[i + argumentBuffer[0]];
                        }
                        addToOutbox(uint8_t(checksumOut));
                        addToOutbox(uint8_t(checksumOut >> 8));

                        //Reset the state machine
                        process = start;
                        break;
                    }
                    case dataReturn:
                        //Store returned data in inbox buffer(this can be reed with FlowSerial::read() )
                        for(uint16_t i = 0; i < argumentBuffer[1]; i++){
                            inboxBuffer[i + inboxBufferAt++] = argumentBuffer[i + 2];
                            inboxAvailable %= inboxBufferSize;
                        }
                        inboxBufferAt += argumentBuffer[1];
                        //Reset the state machine
                        process = start;
                        break;
                    /*
                    case debugInfo:
                        process = idle;
                        break;
                    case debugStateCommand:
                        process = idle;
                        break;
                    case transmissionError:
                        process = idle;
                        break;
                    */
                    default:
                        //In case of not used state reset the state machine
                        process = start;
                }
        default:
            process = start;
    }
    
    /*
    if(millis() - timeoutTime > 150){
        process = idle;
        return -1;
    }
    */
    return 1;
}
uint8_t FlowSerial::read(){
    uint8_t charOut = inboxBuffer[inboxBufferAt - inboxAvailable];
    inboxAvailable--;
    return charOut;
}
uint8_t FlowSerial::available(){
    return inboxAvailable;
}
void FlowSerial::transmissionError(bool resend){
    addToOutbox(0xAA);
    addToOutbox(0x06);
    if(resend == true){
        addToOutbox(0x01);
        addToOutbox(0xb1);
    }
    else{
        addToOutbox(0x00);
        addToOutbox(0xb0);
    }
    addToOutbox(0x00);
}
/*
void FlowSerial::sendDebugInfo(String text, uint16_t textLength){
    addToOutbox(0xAA);
    addToOutbox(0x04);
    addToOutbox(textLength);
    uint16_t serialSum = 0xAE + textLength;
    for(uint16_t i = 0; i < textLength; i++){
        addToOutbox(text[i]);
        serialSum += text[i];
    }
    addToOutbox(uint8_t(serialSum));
    addToOutbox(uint8_t(serialSum >> 8));
}
*/
void FlowSerial::write(uint8_t address, uint8_t out[], uint16_t quantity){
    addToOutbox(0xAA);
    addToOutbox(0x02);
    addToOutbox(address);
    addToOutbox(quantity);
    uint16_t serialSum = 0xAC + address + quantity;
    for(uint16_t i = 0; i < quantity; i++){
        addToOutbox(out[i]);
        serialSum += out[i];
    }
    addToOutbox(uint8_t(serialSum));
    addToOutbox(uint8_t(serialSum >> 8));
}
void FlowSerial::write(uint8_t address, uint8_t out){
    addToOutbox(0xAA);
    addToOutbox(0x02);
    addToOutbox(address);
    addToOutbox(0x01);
    uint16_t serialSum = 0xAD + address + out;
    addToOutbox(out);
    addToOutbox(uint8_t(serialSum));
    addToOutbox(uint8_t(serialSum >> 8));
    //Send the data
}
void FlowSerial::addToOutbox(uint8_t out){
    outboxBuffer[outboxBufferAt++];
    outboxBufferAt %= outboxBufferSize;
    outboxPending++;
}

uint16_t FlowSerial::outboxAvailable(){
    return outboxPending;
}

uint8_t FlowSerial::outboxNextOut(){
    if(outboxBufferAt - outboxPending < 0){
        return outboxBuffer[outboxBufferSize + outboxBufferAt - outboxPending--];
    }
    else{
        return outboxBuffer[outboxBufferAt - outboxPending--];
    }
}

FlowSerial flowSerial;
