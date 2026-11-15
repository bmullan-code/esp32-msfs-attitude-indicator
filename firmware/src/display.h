// ST7701 480x480 RGB panel with two PSRAM frame buffers, swapped in sync with refresh.
#pragma once
#include <stdint.h>

namespace display {
bool begin();                 // panel init, RGB interface, backlight
uint16_t* backBuffer();       // draw the next frame here (480*480 RGB565)
void present();               // show the back buffer at the next refresh, then swap
void setBacklight(uint8_t percent);
}
