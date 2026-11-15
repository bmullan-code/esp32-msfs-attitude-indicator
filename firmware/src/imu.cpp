#include "imu.h"
#include <Arduino.h>
#include <Wire.h>

namespace imu {

static constexpr uint8_t ADDR = 0x6B;
static constexpr uint8_t REG_WHO_AM_I = 0x00;
static constexpr uint8_t REG_CTRL1 = 0x02;    // interface: address auto-increment
static constexpr uint8_t REG_CTRL2 = 0x03;    // accelerometer range and data rate
static constexpr uint8_t REG_CTRL5 = 0x06;    // low-pass filters
static constexpr uint8_t REG_CTRL7 = 0x08;    // sensor enable
static constexpr uint8_t REG_AX_L  = 0x35;
static constexpr float   SCALE_4G  = 4.0f / 32768.0f;
static bool present = false;

static bool writeReg(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t reg, uint8_t* buf, int n) {
    Wire.beginTransmission(ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(ADDR, (uint8_t)n) != n) return false;
    for (int i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

bool begin() {
    uint8_t id = 0;
    if (!readRegs(REG_WHO_AM_I, &id, 1) || id != 0x05) return present = false;
    writeReg(REG_CTRL1, 0x40);            // auto-increment, internal oscillator on
    writeReg(REG_CTRL2, 0x16);            // +-4 g, 125 Hz
    writeReg(REG_CTRL5, 0x01);            // accelerometer low-pass filter on
    writeReg(REG_CTRL7, 0x01);            // accelerometer only
    return present = true;
}

bool readAccel(Vec3& g) {
    if (!present) return false;
    uint8_t b[6];
    if (!readRegs(REG_AX_L, b, 6)) return false;
    g.x = (int16_t)(b[1] << 8 | b[0]) * SCALE_4G;
    g.y = (int16_t)(b[3] << 8 | b[2]) * SCALE_4G;
    g.z = (int16_t)(b[5] << 8 | b[4]) * SCALE_4G;
    return true;
}

}  // namespace imu
