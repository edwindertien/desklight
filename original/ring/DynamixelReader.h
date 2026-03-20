#ifndef __DYNAMIXEL_READER_H__
#define __DYNAMIXEL_READER_H__

#define DYNAMIXEL_BUFFER_SIZE (64) /* Just a lot so that it will not overflow */
#define DYNAMIXEL_RETURN_SIZE 32 
#define BOARD_ID 25
#define MEM_LENGTH 0x34

#define toggle(pin) digitalWrite(pin, !digitalRead(pin)) 
#define  outb(addr, data)  addr = (data)
#define inb(addr)   (addr)
#define BV(bit)     (1<<(bit))
#define cbi(reg,bit)  reg &= ~(BV(bit))
#define sbi(reg,bit)  reg |= (BV(bit))

#define LEDgreen 13


// Prototype for the function we need to call from outside
void DynamixelPoll(void);
void nudgeTimeOut(void);
int getTimeOut(void);

struct DynamixelRAM {
// note, Arduino compiler stores bytes LSB first (so the lowest byte (LSB) of 'int model' is stored on place 0, the MSB in place 1)
  int model; //0x00
  byte version; //0x02
  byte ID;
  byte baudrate;
  byte returnDelay;
  int angleLimitCW; //0x06
  int angleLimitCCW;
  byte dummy4; //0x0A is empty
  byte tempLimitHigh;//0x0B
  byte voltageLimitLow;
  byte voltageLimitHigh;
  int maxTorque; //0x0E
  byte statusReturn;//0x10
  byte alarmLED;
  byte alarmShutdown; //0x12
////----- end of EEPROM memory -----////
  int springOffset; //0x13
  int angleOffset; //0x15
  byte springConstant; //0x17
////---- gap in memory space for future eeprom//
  byte torqueEnable; //0x18
  byte LED;
  byte torqueGain; // 0x1A
  byte angleGain; // 0x1B
  byte speedGain; //0x1C
  byte complianceSlopeCCW; //0x1D // was byte
  int angleSetpoint; //0x1E
  int speedSetpoint; //0x20
  int torqueSetpoint; //0x22
  int currentDistance;   //0x24 - odometry
  int currentSpeed;      //0x26 - driving speed
  int currentDriveLoad;  //0x28 - drive motor load
  int currentAngle;      //0x2A - module angle
  int currentAngleLoad;  //0x2C - bend motor load
  int currentTorque;     //0x2E - clamping force
  byte bendControlMode;      //0x30
  byte driveControlMode; //0x31
  byte currentVoltage;
  byte currentTemperature;
  byte registered;
  byte moving;
  byte lock;
  int punch;
};

#endif

