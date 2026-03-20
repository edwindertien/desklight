#pragma once
// ============================================================
//  DynamixelReader.h
//  RS-485 / Dynamixel protocol driver for MEGA ADK
//  UART2 @ 1 000 000 bd, pin 22 = TX-enable (high = transmit)
// ============================================================

#define DYNAMIXEL_BUFFER_SIZE 64

// ---- low-level bit helpers (kept for compatibility) --------
#define toggle(pin)       digitalWrite(pin, !digitalRead(pin))
#define outb(addr, data)  (addr) = (data)
#define inb(addr)         (addr)
#define BV(bit)           (1 << (bit))
#define cbi(reg, bit)     (reg) &= ~(BV(bit))
#define sbi(reg, bit)     (reg) |= (BV(bit))

// ---- public API --------------------------------------------
void DynamixelPoll(void);
void nudgeTimeOut(void);
int  getTimeOut(void);

void getValue(int ID);
void getValueFrom(int ID, unsigned char address, unsigned char length);
void DynamixelWriteTwoInts(int id, int address, int value1, int value2);
void DynamixelWriteByte(int id, int address, int value);
void DynamixelWrite(int id, int address, int value);

// ---- register map (MX-28 compatible) -----------------------
struct DynamixelRAM {
    int   model;               // 0x00
    byte  version;             // 0x02
    byte  ID;
    byte  baudrate;
    byte  returnDelay;
    int   angleLimitCW;        // 0x06
    int   angleLimitCCW;
    byte  dummy4;              // 0x0A
    byte  tempLimitHigh;       // 0x0B
    byte  voltageLimitLow;
    byte  voltageLimitHigh;
    int   maxTorque;           // 0x0E
    byte  statusReturn;        // 0x10
    byte  alarmLED;
    byte  alarmShutdown;       // 0x12
    // ----- end EEPROM -----
    int   springOffset;        // 0x13
    int   angleOffset;         // 0x15
    byte  springConstant;      // 0x17
    byte  torqueEnable;        // 0x18
    byte  LED;
    byte  torqueGain;          // 0x1A
    byte  angleGain;           // 0x1B  ← P-gain (reg 28 / 0x1C on MX-28)
    byte  speedGain;           // 0x1C
    byte  complianceSlopeCCW;  // 0x1D
    int   angleSetpoint;       // 0x1E  ← goal position (reg 30)
    int   speedSetpoint;       // 0x20
    int   torqueSetpoint;      // 0x22
    int   currentDistance;     // 0x24
    int   currentSpeed;        // 0x26
    int   currentDriveLoad;    // 0x28
    int   currentAngle;        // 0x2A
    int   currentAngleLoad;    // 0x2C
    int   currentTorque;       // 0x2E
    byte  bendControlMode;     // 0x30
    byte  driveControlMode;    // 0x31
    byte  currentVoltage;
    byte  currentTemperature;
    byte  registered;
    byte  moving;
    byte  lock;
    int   punch;
};
