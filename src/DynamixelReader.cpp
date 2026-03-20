// ============================================================
//  DynamixelReader.cpp
//  RS-485 / Dynamixel protocol driver
//  Hardware: Serial2 @ 1 000 000 bd, pin 22 = TX-enable
// ============================================================
#include <Arduino.h>
#include "DynamixelReader.h"

// forward declaration — provided by the main sketch
void ProcessDynamixelData(const unsigned char ID,
                          const int dataLength,
                          const unsigned char* const Data);

volatile int timeOutValue;

// ---- receive state machine ----------------------------------
typedef enum {
    WaitingForFirstHeaderByte,
    WaitingForSecondHeaderByte,
    WaitingForIDByte,
    WaitingForDataLengthByte,
    WaitingForRestOfMessage
} e_receive_state;

void DynamixelPoll() {
    static e_receive_state  receiveState = WaitingForFirstHeaderByte;
    static unsigned char    c;
    static uint8_t          bytesReceived        = 0;
    static unsigned char    addressBuffer        = 0;
    static unsigned char    data[DYNAMIXEL_BUFFER_SIZE];
    static unsigned char    dataLengthBuffer     = 0;
    static unsigned int     checksumBuffer       = 0;

    while (Serial2.available() > 0) {
        c = Serial2.read();
        switch (receiveState) {
        case WaitingForFirstHeaderByte:
            if (c == 0xFF) receiveState = WaitingForSecondHeaderByte;
            break;

        case WaitingForSecondHeaderByte:
            receiveState = (c == 0xFF) ? WaitingForIDByte
                                       : WaitingForFirstHeaderByte;
            break;

        case WaitingForIDByte:
            addressBuffer = c;
            checksumBuffer = c;
            receiveState = WaitingForDataLengthByte;
            break;

        case WaitingForDataLengthByte:
            dataLengthBuffer = c;
            bytesReceived    = 0;
            checksumBuffer  += c;
            receiveState     = WaitingForRestOfMessage;
            break;

        case WaitingForRestOfMessage:
            if (bytesReceived < DYNAMIXEL_BUFFER_SIZE)
                data[bytesReceived] = c;
            checksumBuffer += c;
            bytesReceived++;
            if (bytesReceived == (dataLengthBuffer - 1)) {
                if ((checksumBuffer & 0xFF) == 0xFF &&
                    bytesReceived < DYNAMIXEL_BUFFER_SIZE) {
                    timeOutValue = 0;
                    ProcessDynamixelData(addressBuffer, dataLengthBuffer, data);
                }
                receiveState = WaitingForFirstHeaderByte;
            }
            break;
        }
    }
}

// ---- write helpers -----------------------------------------
static inline void txBegin() { digitalWrite(22, HIGH); }
static inline void txEnd()   { Serial2.flush(); digitalWrite(22, LOW); }

void DynamixelWrite(int id, int address, int value) {
    byte lo  = (byte)(value & 0xFF);
    byte hi  = (byte)(value >> 8);
    byte cks = ~(byte)(id + 0x05 + 0x03 + address + lo + hi);
    unsigned char buf[] = { 0xFF, 0xFF,
        (byte)id, 0x05, 0x03,
        (byte)address, lo, hi, cks };
    txBegin();
    Serial2.write(buf, 9);
    txEnd();
}

void DynamixelWriteTwoInts(int id, int address, int v1, int v2) {
    byte lo1 = (byte)(v1 & 0xFF), hi1 = (byte)(v1 >> 8);
    byte lo2 = (byte)(v2 & 0xFF), hi2 = (byte)(v2 >> 8);
    byte cks = ~(byte)(id + 0x08 + 0x03 + address + lo1 + hi1 + lo2 + hi2);
    unsigned char buf[] = { 0xFF, 0xFF,
        (byte)id, 0x08, 0x03,
        (byte)address, lo1, hi1, lo2, hi2, cks };
    txBegin();
    Serial2.write(buf, 11);
    txEnd();
}

void DynamixelWriteByte(int id, int address, int value) {
    byte cks = ~(byte)(id + 0x04 + 0x03 + address + (byte)value);
    unsigned char buf[] = { 0xFF, 0xFF,
        (byte)id, 0x04, 0x03,
        (byte)address, (byte)value, cks };
    txBegin();
    Serial2.write(buf, 8);
    txEnd();
}

void getValue(int ID) {
    int  cks = ~(ID + 0x05 + 0x02 + 0x24 + 12);
    unsigned char buf[] = { 0xFF, 0xFF,
        (byte)ID, 0x05, 0x02, 0x24, 12, (byte)cks };
    txBegin();
    Serial2.write(buf, 8);
    txEnd();
}

void getValueFrom(int ID, unsigned char address, unsigned char length) {
    int  cks = ~(ID + 0x05 + 0x02 + address + length);
    unsigned char buf[] = { 0xFF, 0xFF,
        (byte)ID, 0x05, 0x02, address, length, (byte)cks };
    txBegin();
    Serial2.write(buf, 8);
    txEnd();
}

int  getTimeOut()    { return timeOutValue; }
void nudgeTimeOut()  { timeOutValue++; }
