// MSFS attitude indicator for the Waveshare ESP32-S3-Touch-LCD-2.1.
//
// Core 0 runs the render task, which draws the instrument and swaps frame buffers in sync
// with refresh. Core 1 runs the Arduino loop (serial link, touch, BOOT button) and the
// display's bounce-buffer interrupt, which is pinned to the core that created the panel.
//
// Modes
//   DEMO     no data received since power-up (or toggled by double-tap): self-test sweep
//   LIVE     samples arriving from the PC bridge
//   NO DATA  samples stopped for more than a second: the card freezes
//   OFF      the bridge reports the instrument unpowered or failed
//
// Controls
//   swipe up / down   raise / lower the miniature airplane (the adjustment knob)
//   long press        centre the miniature airplane
//   double tap        toggle demo mode
//   BOOT button       cycle backlight brightness

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <esp_heap_caps.h>
#include "board.h"
#include "expander.h"
#include "display.h"
#include "touch.h"
#include "renderer.h"
#include "simlink.h"
#include "settings.h"

static constexpr uint32_t STALE_MS        = 1000;
static constexpr float    SMOOTH_TAU_S    = 0.06f;  // display lag behind the sim
static constexpr int      OFFSET_STEP     = 6;
static constexpr int      OFFSET_LIMIT    = 48;
static constexpr uint32_t SAVE_DELAY_MS   = 2000;
static const uint8_t      BRIGHTNESS_STEPS[] = {20, 40, 60, 80, 100};

static void renderTask(void*);

// Shared between the input task and the renderer.
static portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
static Settings cfg;
static volatile bool demoForced = false;
static volatile uint32_t settingsChangedAt = 0;

// ------------------------------------------------------------------ input (core 1)
static void onGesture(Gesture g) {
    portENTER_CRITICAL(&stateMux);
    switch (g) {
        case Gesture::SwipeUp:
            cfg.airplaneOffset = min(OFFSET_LIMIT, cfg.airplaneOffset + OFFSET_STEP);
            break;
        case Gesture::SwipeDown:
            cfg.airplaneOffset = max(-OFFSET_LIMIT, cfg.airplaneOffset - OFFSET_STEP);
            break;
        case Gesture::LongPress:
            cfg.airplaneOffset = 0;
            break;
        case Gesture::DoubleClick:
            demoForced = !demoForced;
            break;
        default:
            portEXIT_CRITICAL(&stateMux);
            return;
    }
    settingsChangedAt = millis();
    portEXIT_CRITICAL(&stateMux);
}

static void cycleBrightness() {
    int next = 0;
    for (int i = 0; i < (int)sizeof(BRIGHTNESS_STEPS); i++)
        if (BRIGHTNESS_STEPS[i] > cfg.brightness) { next = i; break; }
    if (cfg.brightness >= BRIGHTNESS_STEPS[sizeof(BRIGHTNESS_STEPS) - 1]) next = 0;
    cfg.brightness = BRIGHTNESS_STEPS[next];
    display::setBacklight(cfg.brightness);
    settingsChangedAt = millis();
}

static void pollInputs() {
    static uint32_t lastTouch = 0;
    static bool buttonWasDown = false;
    static uint32_t buttonChanged = 0;
    {
        simlink::poll();
        const uint32_t now = millis();

        if (now - lastTouch >= 15) {
            lastTouch = now;
            const Gesture g = touch::poll();
            if (g != Gesture::None) onGesture(g);
        }

        const bool down = digitalRead(PIN_BOOT_BUTTON) == LOW;
        if (down != buttonWasDown && now - buttonChanged > 30) {
            buttonChanged = now;
            buttonWasDown = down;
            if (down) cycleBrightness();
        }

        if (settingsChangedAt && now - settingsChangedAt > SAVE_DELAY_MS) {
            settingsChangedAt = 0;
            settings::save(cfg);
        }
    }
}

// ------------------------------------------------------------------ rendering (core 0)
static float shownPitch = 0, shownBank = 0;

static float wrap180(float a) {
    while (a > 180) a -= 360;
    while (a < -180) a += 360;
    return a;
}

static void scanI2c() {
    Serial.print("I2C devices:");
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a);
    }
    Serial.println();
}

void setup() {
    simlink::begin();
    // Give a USB host a moment to attach so the boot log is visible.
    delay(1500);
    Serial.println("\nattitude indicator starting");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    scanI2c();
    Serial.printf("expander %s\n", expander::begin() ? "ok" : "FAILED");   // also silences the buzzer
    pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);

    cfg = settings::load();
    renderer::begin();
    if (!display::begin()) Serial.println("ERROR: display init failed");
    touch::begin();
    xTaskCreatePinnedToCore(renderTask, "render", 8192, nullptr, 3, nullptr, 0);
    Serial.printf("attitude indicator ready (cpu %lu MHz, free internal %u KB)\n", (unsigned long)getCpuFrequencyMhz(), (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));
}

static void renderFrame() {
    static uint32_t lastMs = millis();
    static uint32_t frames = 0, fpsMark = millis();
    static uint32_t drawUs = 0;
    static bool backlightOn = false;

    const uint32_t now = millis();
    const float dt = (now - lastMs) / 1000.0f;
    lastMs = now;

    simlink::Sample s;
    const bool everReceived = simlink::latest(s);
    const bool fresh = everReceived && (now - s.receivedMs) < STALE_MS;

    AttitudeView view{};
    float targetPitch = shownPitch, targetBank = shownBank;
    if (demoForced || !everReceived) {
        const float t = now / 1000.0f;
        targetPitch = 15.0f * sinf(t * 0.50f);
        targetBank = 50.0f * sinf(t * 0.31f);
        view.flag = Flag::Demo;
    } else if (!fresh) {
        view.flag = Flag::NoData;                 // hold the last attitude
    } else {
        targetPitch = s.pitch;
        targetBank = s.bank;
        const bool connected = s.status & simlink::STATUS_CONNECTED;
        if (!connected) view.flag = Flag::NoData;
        else if (!(s.status & simlink::STATUS_VALID)) view.flag = Flag::Off;
        else view.flag = Flag::None;
    }

    // Exponential smoothing towards the target, the short way round for bank.
    const float k = 1.0f - expf(-dt / SMOOTH_TAU_S);
    shownPitch += (targetPitch - shownPitch) * k;
    shownBank = wrap180(shownBank + wrap180(targetBank - shownBank) * k);

    portENTER_CRITICAL(&stateMux);
    view.airplaneOffset = cfg.airplaneOffset;
    portEXIT_CRITICAL(&stateMux);
    view.pitchDeg = shownPitch;
    view.bankDeg = shownBank;

    const uint32_t t0 = micros();
    renderer::draw(display::backBuffer(), view);
    drawUs += micros() - t0;
    display::present();

    if (!backlightOn) {                           // light up only once a frame is showing
        display::setBacklight(cfg.brightness);
        backlightOn = true;
    }

    frames++;
    if (now - fpsMark >= 5000) {
        Serial.printf("fps %.1f  draw %.1f ms  samples ok %lu bad %lu  pitch %.1f bank %.1f  %s\n",
                      frames * 1000.0f / (now - fpsMark), drawUs / 1000.0f / frames,
                      (unsigned long)simlink::goodCount(),
                      (unsigned long)simlink::badCount(), shownPitch, shownBank,
                      view.flag == Flag::Demo ? "DEMO" : view.flag == Flag::NoData ? "NO DATA"
                      : view.flag == Flag::Off ? "OFF" : "LIVE");
        frames = 0;
        drawUs = 0;
        fpsMark = now;
    }
}

static void renderTask(void*) {
    for (;;) renderFrame();
}

void loop() {
    pollInputs();
    delay(2);
}
