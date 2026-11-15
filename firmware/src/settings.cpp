#include "settings.h"
#include <Preferences.h>

namespace settings {

Settings load() {
    Settings s;
    Preferences p;
    if (p.begin("attitude", true)) {
        s.airplaneOffset = p.getChar("offset", s.airplaneOffset);
        s.brightness = p.getUChar("bright", s.brightness);
        p.end();
    }
    return s;
}

void save(const Settings& s) {
    Preferences p;
    if (!p.begin("attitude", false)) return;
    p.putChar("offset", s.airplaneOffset);
    p.putUChar("bright", s.brightness);
    p.end();
}

}  // namespace settings
