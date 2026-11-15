// User settings kept in flash (NVS).
#pragma once
#include <stdint.h>

struct Settings {
    int8_t  airplaneOffset = 0;    // miniature airplane height, pixels, up positive
    uint8_t brightness = 80;       // backlight percent
};

namespace settings {
Settings load();
void save(const Settings& s);
}
