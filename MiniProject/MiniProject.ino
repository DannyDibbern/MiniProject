// Libraries
#include <Arduino.h>
#include <Dynamixel_MiniProject.h>
#include <SoftwareSerial.h>

// Dynamixel_Serial
#define SERVO_ControlPin 0x02       // Control pin of buffer chip, NOTE: this does not matter becasue we are not using a half to full contorl buffer.
#define SERVO_SET_Baudrate 57600    // Baud rate speed which the Dynamixel will be set too (57600)

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  
  Serial.flush();                                         // Clear the serial buffer of garbage data before running the code.
  mySerial.begin(SERVO_SET_Baudrate);                     // We now need to set Ardiuno to the new Baudrate speed 57600
  Serial.begin(57600);                                    // Start serial communication on baudrate 57600
  Dynamixel.begin(mySerial);                              // Calling mySerial function which sets 10 pin as the 2nd RX serial pin, and sets pin 11 as the 2nd TX serial pin
  Dynamixel.setDirectionPin(SERVO_ControlPin);            // Optional. Set direction control pin

  Dynamixel.setHoldingTorque(0x02, false);
  Dynamixel.setPositionPGain(0x02, 1000);
  Dynamixel.setVelocityPGain(0x02, 1000);
}

int TEMP_TOO_HIGH = 20;
int currentTemp = 0;
int startTime = 0;
int finishTime = 0;
int difTime = 0;

void loop() {
  startTime = millis();

    currentTemp = Dynamixel.getTemperature(0x02);

    if(currentTemp > TEMP_TOO_HIGH) {
      Dynamixel.factoryReset(0x02, 0x02);
    }

  Serial.print("Time in milliseconds: ");
  Serial.println(startTime);

  finishTime = millis();
  difTime = finishTime - startTime;
  
  delay(100 - difTime);
}
