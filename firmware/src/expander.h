// TCA9554 8-bit I/O expander (all pins used as outputs on this board).
#pragma once
#include <stdint.h>

namespace expander {
bool begin();                       // all pins outputs, buzzer off, everything else high
void write(int exio, bool level);   // exio is 1..8
}
