#include "touch.h"
#include "board.h"
#include "expander.h"
#include <Arduino.h>
#include <Wire.h>

namespace touch {

static constexpr uint8_t ADDR = 0x15;
static constexpr uint8_t REG_GESTURE = 0x01;
static constexpr uint8_t REG_DISABLE_AUTOSLEEP = 0xFE;
static uint8_t lastGesture = 0;
static volatile bool pending = false;

// The CST820 pulls INT low when it has new touch data. It does not answer on I2C until it
// has been touched, so it is only read after INT fires.
static void IRAM_ATTR onInt() { pending = true; }

bool begin() {
    pinMode(PIN_TOUCH_INT, INPUT_PULLUP);
    expander::write(EXIO_TOUCH_RESET, false);
    delay(10);
    expander::write(EXIO_TOUCH_RESET, true);
    delay(50);
    attachInterrupt(PIN_TOUCH_INT, onInt, FALLING);
    return true;
}

// The chip only answers on I2C while it is awake (after a touch), so auto-sleep is
// disabled on the first successful read rather than at start-up.
static bool autoSleepDisabled = false;
static void disableAutoSleep() {
    Wire.beginTransmission(ADDR);
    Wire.write(REG_DISABLE_AUTOSLEEP);
    Wire.write(0xFF);
    autoSleepDisabled = Wire.endTransmission() == 0;
}

Gesture poll() {
    if (!pending && digitalRead(PIN_TOUCH_INT) == HIGH) return Gesture::None;
    pending = false;

    Wire.beginTransmission(ADDR);
    Wire.write(REG_GESTURE);
    if (Wire.endTransmission(true) != 0) return Gesture::None;
    if (Wire.requestFrom(ADDR, (uint8_t)2) != 2) return Gesture::None;
    const uint8_t gesture = Wire.read();
    Wire.read();                                   // touch point count (unused)
    if (!autoSleepDisabled) disableAutoSleep();

    // The controller keeps reporting a gesture while the finger is down; report each once.
    if (gesture == lastGesture) return Gesture::None;
    lastGesture = gesture;
    return static_cast<Gesture>(gesture);
}

}  // namespace touch
