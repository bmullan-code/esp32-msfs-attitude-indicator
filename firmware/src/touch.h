// CST820 capacitive touch controller: single touch plus built-in gesture detection.
#pragma once
#include <stdint.h>

enum class Gesture : uint8_t {
    None = 0x00, SwipeUp = 0x01, SwipeDown = 0x02, SwipeLeft = 0x03, SwipeRight = 0x04,
    Click = 0x05, DoubleClick = 0x0B, LongPress = 0x0C,
};

namespace touch {
bool begin();          // resets the controller through the expander
Gesture poll();        // returns a gesture once, when it first appears
}
