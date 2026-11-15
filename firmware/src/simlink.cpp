#include "simlink.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

namespace simlink {

static constexpr const char* IDENT = "ATTITUDE-IND,1,touch-lcd-2.1";

struct LineReader {
    Stream* port;
    char buf[96];
    int len = 0;
};

static LineReader readers[2];
static Sample current;
static bool haveSample = false;
static uint32_t good = 0, bad = 0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static void handleLine(LineReader& r) {
    char* s = r.buf;
    if (strcmp(s, "?") == 0) {
        r.port->println(IDENT);
        return;
    }
    if (s[0] != '$') return;
    char* star = strchr(s, '*');
    if (!star || star[1] == 0 || star[2] == 0) { bad++; return; }
    uint8_t sum = 0;
    for (char* p = s + 1; p < star; p++) sum ^= (uint8_t)*p;
    const int hi = hexVal(star[1]), lo = hexVal(star[2]);
    if (hi < 0 || lo < 0 || sum != (uint8_t)(hi * 16 + lo)) { bad++; return; }
    *star = 0;

    float pitch, bank, slip;
    int status;
    if (sscanf(s, "$ATT,%f,%f,%f,%d", &pitch, &bank, &slip, &status) != 4) { bad++; return; }

    Sample next;
    next.pitch = pitch;
    next.bank = bank;
    next.slip = slip;
    next.status = (uint8_t)status;
    next.receivedMs = millis();
    portENTER_CRITICAL(&mux);
    current = next;
    haveSample = true;
    good++;
    portEXIT_CRITICAL(&mux);
}

void begin() {
    // USB port: native USB serial. UART port: the CH343 bridge on GPIO43/44.
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);          // never block when no host is listening
    Serial0.begin(115200);
    readers[0].port = &Serial;
    readers[1].port = &Serial0;
}

void poll() {
    for (LineReader& r : readers) {
        while (r.port->available()) {
            const char c = (char)r.port->read();
            if (c == '\r') continue;
            if (c == '\n') {
                r.buf[r.len] = 0;
                if (r.len) handleLine(r);
                r.len = 0;
            } else if (r.len < (int)sizeof(r.buf) - 1) {
                r.buf[r.len++] = c;
            } else {
                r.len = 0;                 // overlong line: discard
                bad++;
            }
        }
    }
}

bool latest(Sample& out) {
    portENTER_CRITICAL(&mux);
    out = current;
    const bool ok = haveSample;
    portEXIT_CRITICAL(&mux);
    return ok;
}

uint32_t goodCount() { return good; }
uint32_t badCount() { return bad; }

}  // namespace simlink
