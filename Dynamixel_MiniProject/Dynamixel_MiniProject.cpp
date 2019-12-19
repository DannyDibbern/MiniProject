/*

Version 2.2

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 */

#include "Dynamixel_MiniProject.h"

//##############################################################################
//############################ Public Methods ##################################
//##############################################################################

void DynamixelClass::begin(long baud){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
    Serial1.begin(baud);  // Set up Serial for Leonardo and Mega
    _serial = &Serial1;
#else
    Serial.begin(baud);   // Set up Serial for all others (Uno, etc)
    _serial = &Serial;
#endif

}

void DynamixelClass::begin(HardwareSerial &HWserial, long baud){

    HWserial.begin(baud); // Set up Serial for a specified Serial object
    _serial = &HWserial;

}

void DynamixelClass::begin(Stream &serial){

    _serial = &serial;  // Set a reference to a specified Stream object (Hard or Soft Serial)

}

void DynamixelClass::end(){

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__)
    Serial1.end();
#else
    Serial.end();
#endif

}

//Setters
void DynamixelClass::setDirectionPin(unsigned char D_Pin){
    Direction_Pin = D_Pin;
    pinMode(Direction_Pin,OUTPUT);
}

void DynamixelClass::setHoldingTorque(unsigned char ID, bool Set) {
  unsigned char arr[1] = {Set};
  writeN(ID, 0x40, arr, 1);
}

void DynamixelClass::setPositionPGain(unsigned char ID, unsigned int gain) {

    gain %= 16383;

    unsigned char arr[] = {
        (gain & 0xFF),
        (gain & 0xFF00) >> 8,
        (gain & 0xFF0000) >> 16,
        (gain & 0xFF000000) >> 24
    };

    writeN(ID, 0x54, arr, 4);
}

void DynamixelClass::setVelocityPGain(unsigned char ID, unsigned int gain) {

    gain %= 16383;

    unsigned char arr[] = {
        (gain & 0xFF),
        (gain & 0xFF00) >> 8,
        (gain & 0xFF0000) >> 16,
        (gain & 0xFF000000) >> 24
    };

    writeN(ID, 0x4E, arr, 4);
}

//Getters
int DynamixelClass::getTemperature(unsigned char ID) {

    clearRXbuffer();
    readN(ID, 0x92, 4);                   //Read from adress 0x92 (Present Temperature), byte size 4
    getParameters();                      //Filters parameters from ReturnPacket

    int temp;
    temp = (data[2] << 8) | data[1];       //Converting two information bytes into a integer (position data)

    //Debug information
    Serial.print("Temperature of ID: ");
    Serial.print(data[0]);
    Serial.print(" is ");
    Serial.println(temp);

    return temp;
}

int DynamixelClass::getPositionPGain(unsigned char ID) {

    clearRXbuffer();
    readN(ID, 0x54, 4);                   //Read from adress 0x54 (P Gain of Position), byte size 4
    getParameters();                      //Filters parameters from ReturnPacket

    int ppgain;
    ppgain = (data[2] << 8) | data[1];       //Converting two information bytes into a integer (position data)

    //Debug information
    Serial.print("Position P Gain of ID: ");
    Serial.print(data[0]);
    Serial.print(" is ");
    Serial.println(ppgain);

    return ppgain;
}

int DynamixelClass::getVelocityPGain(unsigned char ID) {

    clearRXbuffer();
    readN(ID, 0x4E, 4);                   //Read from adress 0x4E (P Gain of Velocity), byte size 4
    getParameters();                      //Filters parameters from ReturnPacket

    int vpgain;
    vpgain = (data[2] << 8) | data[1];       //Converting two information bytes into a integer (position data)

    //Debug information
    Serial.print("Velocity P Gain of ID: ");
    Serial.print(data[0]);
    Serial.print(" is ");
    Serial.println(vpgain);

    return vpgain;
}

bool DynamixelClass::getHoldingTorque(unsigned char ID) {

    clearRXbuffer();
    readN(ID, 0x40, 1);                   //Read from adress 0x40 (Torque Enable), byte size 1
    getParameters();                      //Filters parameters from ReturnPacket

    bool set;
    set = data[2];       //Converting two information bytes into a integer (position data)

    //Debug information
    Serial.print("Holding Torque of ID: ");
    Serial.print(data[0]);
    Serial.print(" is ");
    Serial.println(set);

    return set;
}


//Instruction Packets

void DynamixelClass::factoryReset(unsigned char ID, unsigned short addr) {

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = (4 & 0xFF); //length
    Instruction_Packet_Array[2] = (4 & 0xFF00) >> 8; //length
    Instruction_Packet_Array[3] = 0x06; //Instruction to factory reset
    Instruction_Packet_Array[4] = (addr & 0xFF); //Address 0xFF: Reset all - 0x01: Reset all except ID - 0x02: Reset all except ID and Baudrate

    clearRXbuffer();

    transmitInstructionPacket(4);

    return 0;
}

void DynamixelClass::writeN(unsigned char ID, unsigned short addr, unsigned char *arr, int n){

    n += 5;
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = (n & 0xFF); //length
    Instruction_Packet_Array[2] = (n & 0xFF00) >> 8; //length
    Instruction_Packet_Array[3] = 0x03; //Instruction
    Instruction_Packet_Array[4] = (addr & 0xFF); //address
    Instruction_Packet_Array[5] = (addr & 0xFF00) >> 8; //address

    for (int i = 0; i < n - 5; i++) {
        Instruction_Packet_Array[i+6] = arr[i];
    }

    clearRXbuffer();

    transmitInstructionPacket(n);

}

void DynamixelClass::readN(unsigned char ID, unsigned short addr, int n){

    n += 3;
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = (n & 0xFF); //length of packet
    Instruction_Packet_Array[2] = (n & 0xFF00) >> 8; //length of packet
    Instruction_Packet_Array[3] = 0x02; //Instruction
    Instruction_Packet_Array[4] = (addr & 0xFF); //address
    Instruction_Packet_Array[5] = (addr & 0xFF00) >> 8; //address
    Instruction_Packet_Array[6] = ((n-3) & 0xFF); //data length
    Instruction_Packet_Array[7] = ((n-3) & 0xFF00) >> 8; // data length

    clearRXbuffer();

    transmitInstructionPacket(n);
    readReturnPacket();
}

//##############################################################################
//########################## Private Methods ###################################
//##############################################################################

void DynamixelClass::getParameters(void) {

    int j = 0;
    for (int i = 0; i < 100; i++) {
        //Filtering the parameters from the returnpacket, by searching for the instruction (0x00, 0x55, 0x00)
        if (ReturnPacket[i] == 0x55 && ReturnPacket[i - 1] == 0 && ReturnPacket[i + 1] == 0) {
            //Saving ID
            data[j] = ReturnPacket[i - 3];

            //Saving parameter bytes
            data[j + 1] = ReturnPacket[i + 2];
            data[j + 2] = ReturnPacket[i + 3];

            j += 3;
        }
    }

}

void DynamixelClass::transmitInstructionPacket(int transLen){                                   // Transmit instruction packet to Dynamixel

    if (Direction_Pin > -1){
        digitalWrite(Direction_Pin,HIGH);                                               // Set TX Buffer pin to HIGH
    }

    unsigned char arrLen = transLen+7;
    unsigned char pt[arrLen];

    pt[0] = 0xFF;
    pt[1] = 0xFF;
    pt[2] = 0xFD;
    pt[3] = 0x00;
    int i;
    for (i = 0; i <= transLen; i++) {
      pt[i+4] = Instruction_Packet_Array[i];
    }

    unsigned short crc = update_crc(pt, arrLen-2);

    unsigned char CRC_L = (crc & 0x00FF);
    unsigned char CRC_H = (crc>>8) & 0x00FF;

    i += 4;

    pt[i++] = CRC_L;
    pt[i] = CRC_H;

    for(i = 0; i < arrLen; i++) {
      _serial->write(pt[i]);
    }

    noInterrupts();

#if defined(__AVR_ATmega32U4__) || defined(__MK20DX128__) || defined(__AVR_ATmega2560__) // Leonardo and Mega use Serial1
    if ((UCSR1A & B01100000) != B01100000){                                             // Wait for TX data to be sent
        _serial->flush();
    }

#elif defined(__SAM3X8E__)

    //if(USART_GetFlagStatus(USART1, USART_FLAG_TC) != RESET)
        _serial->flush();
    //}

#else
    if ((UCSR0A & B01100000) != B01100000){                                             // Wait for TX data to be sent
        _serial->flush();
    }

#endif

    if (Direction_Pin > -1){
        digitalWrite(Direction_Pin,LOW);                                                //Set TX Buffer pin to LOW after data has been sent
    }

    interrupts();

    delay(20);

}

void DynamixelClass::readReturnPacket(void){

  int i = 0;
  //Read information when available
  while(_serial->available()>0){
    int incomingbyte;
    incomingbyte = _serial->read();        //Save incomingbyte

    ReturnPacket[i]=incomingbyte;          //Save data in ReturnPacket array
    i++;
  }
}

void DynamixelClass::clearRXbuffer(void){

    while (_serial->read() != -1);  // Clear RX buffer;

}

unsigned short DynamixelClass::update_crc(unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short crc_accum = 0;
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}

DynamixelClass Dynamixel;
