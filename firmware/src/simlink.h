// Receives attitude samples from the PC bridge over USB serial.
//
// Line protocol (ASCII, one sample per line, NMEA-style XOR checksum):
//     $ATT,<pitch>,<bank>,<slip>,<status>*HH\n
//   pitch   degrees, nose up positive
//   bank    degrees, right bank positive
//   slip    turn coordinator ball, -1..1 (reserved, not drawn yet)
//   status  bit0 sim connected, bit1 sim paused, bit2 instrument valid (powered, not failed)
//   HH      XOR of every character between '$' and '*', two hex digits
//
// A line containing just "?" is answered with "ATTITUDE-IND,<protocol>,<board>" so the
// bridge can find the right serial port automatically.
#pragma once
#include <stdint.h>

namespace simlink {

constexpr uint8_t STATUS_CONNECTED = 0x01;
constexpr uint8_t STATUS_PAUSED    = 0x02;
constexpr uint8_t STATUS_VALID     = 0x04;

struct Sample {
    float pitch = 0, bank = 0, slip = 0;
    uint8_t status = 0;
    uint32_t receivedMs = 0;
};

void begin();
void poll();                       // read and parse whatever has arrived
bool latest(Sample& out);          // false until the first valid sample
uint32_t goodCount();
uint32_t badCount();

}  // namespace simlink
