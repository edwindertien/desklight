/////////////////////////////////////////////////////
// Dynamixel compatible version of a neopixel ring
// lamp head for the robot desklight
//
// the lamp responds to the position and velocity 
// commands, mapped to a hue and brightness
// the address (in the DynamixelReader.h) can 
// be changed, but is set at 25
//
// In the current version an Arduino mini is used
// with an external switching regulator and a small
// RS485 adapter pcb
/////////////////////////////////////////////////////

#include "DynamixelReader.h"
#define RS485sr 2
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel ring = Adafruit_NeoPixel(16, 10, NEO_GRB + NEO_KHZ800);

DynamixelRAM mem;
char* memCom = (char*) &mem;
volatile boolean dataValid;

void setup() {
  // put your setup code here, to run once:
    pinMode(RS485sr,OUTPUT);
    pinMode(LEDgreen,OUTPUT);
    //Serial1.begin(1000000);
    Serial.begin(1000000);
//      while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
    ring.begin();
    ring.show();
}

void loop() {
  // put your main code here, to run repeatedly:
  DynamixelPoll();
  if(dataValid) {
    toggle(LEDgreen);
    //Serial.println((mem.angleSetpoint)/8);
    dataValid =0;
    if(torqueSetpoint/4>255) torqueSetpoint = 1023;
    for (uint16_t i = 0; i < torqueSetpoint/(4*ring.numPixels); i++) {
      ring.setPixelColor(i, Wheel((mem.angleSetpoint)/4, 255-(mem.speedSetpoint)/4));
    }
    for (uint16_t i =torqueSetpoint/(4*ring.numPixels); i < ring.numPixels; i++) {
      ring.setPixelColor(i, 0));
    }
    ring.show();
    }
}

void ProcessDynamixelData(const unsigned char ID, const int dataLength, const unsigned char* const Data){
  byte buffer[DYNAMIXEL_RETURN_SIZE];
  toggle(LEDgreen);
  switch(Data[0]){
  case 0x01: //ping
    ReturnDynamixelData(ID,0,0);
  case 0x02: //read
    for(int n=0; n<Data[2];n++){
      buffer[n] = memCom[Data[1]+n];
    }
    dataValid=1;
    delayMicroseconds(mem.returnDelay*2);
    ReturnDynamixelData(ID,Data[2],buffer);
    break; 
  case 0x03: // Write command
    if(Data[1]<MEM_LENGTH && (dataLength-3)<(MEM_LENGTH-Data[1])){// safeguard
      for (int n=0;n<dataLength-3;n++){
        memCom[(byte)Data[1]+n] = (byte)Data[2+n];
      }
      dataValid = 1;
    }
    break;
  case 0x04: //REG WRITE
  case 0x05: //ACTION
  case 0x06: //RESET 
  case 0x83: //SYNC WRITE
  default:
    break;
  }
}

void ReturnDynamixelData(const unsigned char ID, const int dataLength, const unsigned char* const Data){
  unsigned char buffer[DYNAMIXEL_RETURN_SIZE];
  unsigned int checksum = 0;
  unsigned char error = 0;
  buffer[0] = (0xFF);
  buffer[1] = (0xFF);
  buffer[2] =(ID);
  buffer[3] = (dataLength+3); // was dataLegth+2
  buffer[4] = (error);
  if(dataLength>0){
    for (int n = 0; n< (dataLength); n++) { // dataLength = N paramaters + 2
      buffer[5+n] = Data[n];
      checksum += Data[n];
    }
    buffer[5+dataLength] = (~(checksum+ID+error+dataLength+3));
  }
  else{
    buffer[5+dataLength] = (~(checksum+ID+error+dataLength+3));
  }
  // OK, do transmission now: //
  digitalWrite(RS485sr,HIGH); // send data to HOST
  Serial.write(buffer,dataLength+6);
  Serial.flush(); // wait for buffer to empty
  digitalWrite(RS485sr,LOW);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos, byte brightness) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return ring.Color(brightness * (255 - WheelPos * 3) / 256, 0, (brightness * WheelPos * 3) / 256);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return ring.Color(0, (brightness * WheelPos * 3) / 256, brightness * (255 - WheelPos * 3) / 256);
  } else {
    WheelPos -= 170;
    return ring.Color((brightness * WheelPos * 3) / 256, brightness * (255 - WheelPos * 3) / 256, 0);
  }
}
