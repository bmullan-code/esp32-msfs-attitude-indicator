#include "expander.h"
#include "board.h"
#include <Wire.h>

namespace expander {

static constexpr uint8_t ADDR = 0x20;
static constexpr uint8_t REG_OUTPUT = 0x01;
static constexpr uint8_t REG_CONFIG = 0x03;
static uint8_t outputs = 0;

static bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool begin() {
    // Resets, LCD CS and SD CS idle high; buzzer (EXIO8) off.
    outputs = 0x7F;
    bool ok = writeReg(REG_OUTPUT, outputs);
    ok = writeReg(REG_CONFIG, 0x00) && ok;     // 0 = output
    return ok;
}

void write(int exio, bool level) {
    const uint8_t bit = 1u << (exio - 1);
    outputs = level ? (outputs | bit) : (outputs & ~bit);
    writeReg(REG_OUTPUT, outputs);
}

}  // namespace expander
