#ifndef Dynamixel_MiniProject_h
#define Dynamixel_MiniProject_h

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
 #include <pins_arduino.h>
#endif


#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define STATUS_PACKET_TIMEOUT           50
#define STATUS_FRAME_BUFFER             5

class DynamixelClass {
public:
    // Constructor
    DynamixelClass(): Direction_Pin(-1), Status_Return_Value(READ) { }

    void begin(long);
    void begin(HardwareSerial&, long);
    void begin(Stream&);
    void end(void);

    void setDirectionPin(unsigned char D_Pin);
    void setHoldingTorque(unsigned char ID, bool Set);
    void setPositionPGain(unsigned char ID, unsigned int gain);
    void setVelocityPGain(unsigned char ID, unsigned int gain);

    int  getTemperature(unsigned char ID);
    int  getPositionPGain(unsigned char ID);
    int  getVelocityPGain(unsigned char ID);
    bool getHoldingTorque(unsigned char ID);

    void factoryReset(unsigned char ID, unsigned short addr);
    void writeN(unsigned char ID, unsigned short addr, unsigned char *arr, int n);
    void readN(unsigned char ID, unsigned short addr, int n);

    Stream *_serial;

private:
    void getParameters(void);
    void transmitInstructionPacket(int transLen);
    void readReturnPacket(void);
    void clearRXbuffer(void);
    unsigned short update_crc(unsigned char* data_blk_ptr, unsigned short data_blk_size);

    unsigned char   Instruction_Packet_Array[64];   // Array to hold instruction packet data
    unsigned int    ReturnPacket[100];              // Array to hold returned status packet data
    unsigned long   Time_Counter;                   // Timer for time out watchers
    char            Direction_Pin;                  // Pin to control TX/RX buffer chip
    unsigned char   Status_Return_Value;            // Status packet return states ( NON , READ , ALL )
    unsigned int    data[15];                       // Data from ReturnPacket
    int             returndata[5];                  // Data return
};


extern DynamixelClass Dynamixel;

#endif
