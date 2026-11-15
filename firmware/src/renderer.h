// Draws the attitude indicator face into a 480x480 RGB565 frame buffer.
#pragma once
#include <stdint.h>

enum class Flag : uint8_t { None, Demo, NoData, Off };

struct AttitudeView {
    float pitchDeg;        // nose up positive
    float bankDeg;         // right bank positive
    int   airplaneOffset;  // miniature airplane height adjustment, pixels, up positive
    Flag  flag;
};

namespace renderer {
void begin();                                        // pre-renders text masks
void draw(uint16_t* fb, const AttitudeView& view);
}
