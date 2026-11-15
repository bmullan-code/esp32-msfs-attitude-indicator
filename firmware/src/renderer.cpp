#include "renderer.h"
#include "board.h"
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <math.h>
#include <vector>
#include <esp_heap_caps.h>

namespace renderer {

// ------------------------------------------------------------------ look and geometry
static constexpr uint16_t rgb(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static constexpr uint16_t C_SKY      = rgb(48, 146, 226);
static constexpr uint16_t C_GROUND   = rgb(120, 86, 44);
static constexpr uint16_t C_WHITE    = rgb(255, 255, 255);
static constexpr uint16_t C_ORANGE   = rgb(255, 128, 0);
static constexpr uint16_t C_PEDESTAL = rgb(28, 28, 30);
static constexpr uint16_t C_BEZEL    = rgb(52, 54, 58);
static constexpr uint16_t C_RIM      = rgb(10, 10, 12);
static constexpr uint16_t C_RED      = rgb(210, 20, 20);
static constexpr uint16_t C_AMBER    = rgb(255, 176, 0);
static constexpr uint16_t C_BLACK    = 0;

static constexpr float CX = 240.0f, CY = 240.0f;
static constexpr float R_CARD   = 226.0f;   // edge of the moving card
static constexpr float R_LADDER = 150.0f;   // pitch ladder window
static constexpr float PPD      = 6.0f;     // pixels per degree of pitch
static constexpr float DEG      = 0.01745329252f;

struct Pt { float x, y; };                  // math coordinates: origin at centre, y up

// Frames are drawn in horizontal strips in fast internal RAM, then each strip is copied to
// the PSRAM frame buffer in one sequential block. FB is offset so that FB[y * LCD_W + x]
// addresses the strip for rows Y0 <= y < Y1.
static constexpr int STRIP_ROWS = 48;
static uint16_t* strip = nullptr;
static uint16_t* FB = nullptr;
static int Y0 = 0, Y1 = 0;

// ------------------------------------------------------------------ low-level spans
// floorf/ceilf are library calls on this chip; these inline versions are much cheaper.
static inline int ifloor(float x) { int i = (int)x; return i - (x < (float)i); }
static inline int iceil(float x) { int i = (int)x; return i + (x > (float)i); }

static IRAM_ATTR inline void hspan(int y, int x0, int x1, uint16_t c) {
    if (y < Y0 || y >= Y1) return;
    if (x0 < 0) x0 = 0;
    if (x1 > LCD_W - 1) x1 = LCD_W - 1;
    if (x0 > x1) return;
    uint16_t* p = FB + y * LCD_W + x0;
    int n = x1 - x0 + 1;
    // Align, then write two pixels per 32-bit store.
    if (((uintptr_t)p & 2) && n) { *p++ = c; n--; }
    uint32_t c2 = ((uint32_t)c << 16) | c;
    uint32_t* q = (uint32_t*)p;
    for (int i = 0; i < n / 2; i++) q[i] = c2;
    if (n & 1) p[n - 1] = c;
}

static inline IRAM_ATTR float circleHalfWidth(float r, float py) {
    float d = r * r - py * py;
    return d > 0 ? sqrtf(d) : -1.0f;
}

// Fills a convex polygon given in math coordinates, optionally clipped to a centred circle.
static IRAM_ATTR void fillConvex(const Pt* p, int n, uint16_t c, float clipR) {
    float ymin = p[0].y, ymax = p[0].y;
    for (int i = 1; i < n; i++) { ymin = fminf(ymin, p[i].y); ymax = fmaxf(ymax, p[i].y); }
    int sy0 = iceil(CY - ymax - 0.5f), sy1 = ifloor(CY - ymin - 0.5f);
    if (sy0 < Y0) sy0 = Y0;
    if (sy1 > Y1 - 1) sy1 = Y1 - 1;
    for (int sy = sy0; sy <= sy1; sy++) {
        const float py = CY - (sy + 0.5f);
        float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < n; i++) {
            const Pt& a = p[i];
            const Pt& b = p[(i + 1) % n];
            if ((a.y <= py && b.y > py) || (b.y <= py && a.y > py)) {
                const float x = a.x + (py - a.y) * (b.x - a.x) / (b.y - a.y);
                lo = fminf(lo, x);
                hi = fmaxf(hi, x);
            }
        }
        if (lo > hi) continue;
        if (clipR > 0) {
            const float hw = circleHalfWidth(clipR, py);
            if (hw < 0) continue;
            lo = fmaxf(lo, -hw);
            hi = fminf(hi, hw);
            if (lo > hi) continue;
        }
        hspan(sy, iceil(lo + CX - 0.5f), ifloor(hi + CX - 0.5f), c);
    }
}

// ------------------------------------------------------------------ frames of reference
// Card: the moving sky/ground disc. (u right, v up) in pixels, v = pitch * PPD on the ladder.
// It rotates with bank and slides with pitch. Ring: the bank scale, rotates only.
struct Frame { float cb, sb, pitchPx, bankRad; };
static Frame F;

static inline IRAM_ATTR Pt card(float u, float v) {
    const float vr = v - F.pitchPx;
    return {u * F.cb - vr * F.sb, u * F.sb + vr * F.cb};
}
static inline IRAM_ATTR Pt ring(float u, float v) { return {u * F.cb - v * F.sb, u * F.sb + v * F.cb}; }

static IRAM_ATTR void cardRect(float u0, float u1, float v0, float v1, uint16_t c, float clipR) {
    Pt q[4] = {card(u0, v0), card(u1, v0), card(u1, v1), card(u0, v1)};
    fillConvex(q, 4, c, clipR);
}

// A thick segment in card coordinates.
static IRAM_ATTR void cardSegment(float u0, float v0, float u1, float v1, float halfWidth, uint16_t c, float clipR) {
    float du = u1 - u0, dv = v1 - v0;
    const float len = sqrtf(du * du + dv * dv);
    if (len <= 0) return;
    const float nu = -dv / len * halfWidth, nv = du / len * halfWidth;
    Pt q[4] = {card(u0 + nu, v0 + nv), card(u1 + nu, v1 + nv), card(u1 - nu, v1 - nv), card(u0 - nu, v0 - nv)};
    fillConvex(q, 4, c, clipR);
}

// ------------------------------------------------------------------ text masks
struct Mask {
    int w = 0, h = 0;
    std::vector<uint8_t> px;
};

static Mask makeMask(const char* text, const GFXfont* font) {
    // Text wrapping must be off: it is on by default, and the measuring canvas is tiny.
    GFXcanvas1 probe(1, 1);
    probe.setTextWrap(false);
    probe.setFont(font);
    int16_t bx, by;
    uint16_t bw, bh;
    probe.getTextBounds(text, 0, 0, &bx, &by, &bw, &bh);
    Mask m;
    m.w = bw + 2;
    m.h = bh + 2;
    GFXcanvas1 canvas(m.w, m.h);
    canvas.fillScreen(0);
    canvas.setTextWrap(false);
    canvas.setFont(font);
    canvas.setTextColor(1);
    canvas.setCursor(1 - bx, 1 - by);
    canvas.print(text);
    m.px.resize(m.w * m.h);
    for (int y = 0; y < m.h; y++)
        for (int x = 0; x < m.w; x++) m.px[y * m.w + x] = canvas.getPixel(x, y) ? 1 : 0;
    return m;
}

// Draws a mask centred on a point (math coordinates), rotated by `angle` (radians, CCW).
static IRAM_ATTR void drawMask(const Mask& m, Pt centre, float angle, uint16_t c, float clipR) {
    const uint8_t* bits = m.px.data();
    const float hw = m.w * 0.5f, hh = m.h * 0.5f;
    if (angle == 0) {                                   // unrotated: straight copy
        const int sx0 = ifloor(centre.x + CX - hw + 0.5f), sy0 = ifloor(CY - centre.y - hh + 0.5f);
        for (int my = 0; my < m.h; my++) {
            const int sy = sy0 + my;
            if (sy < Y0 || sy >= Y1) continue;
            for (int mx = 0; mx < m.w; mx++) {
                const int sx = sx0 + mx;
                if (sx >= 0 && sx < LCD_W && bits[my * m.w + mx]) FB[sy * LCD_W + sx] = c;
            }
        }
        return;
    }
    const float ca = cosf(angle), sa = sinf(angle);
    // Screen-space extent of the rotated mask.
    const float ex = fabsf(hw * ca) + fabsf(hh * sa), ey = fabsf(hw * sa) + fabsf(hh * ca);
    const int sxa = max(0, ifloor(centre.x + CX - ex)), sxb = min(LCD_W - 1, iceil(centre.x + CX + ex));
    const int sya = max(Y0, ifloor(CY - centre.y - ey)), syb = min(Y1 - 1, iceil(CY - centre.y + ey));
    const float clip2 = clipR * clipR;
    for (int sy = sya; sy <= syb; sy++) {
        const float py = CY - (sy + 0.5f);
        const float dy = py - centre.y;
        const float px0 = sxa + 0.5f - CX;
        const float dx0 = px0 - centre.x;
        // Mask coordinates step linearly along the row.
        float mu = dx0 * ca + dy * sa + hw;
        float mv = hh - (-dx0 * sa + dy * ca);
        float px = px0;
        uint16_t* row = FB + sy * LCD_W;
        for (int sx = sxa; sx <= sxb; sx++, mu += ca, mv += sa, px += 1.0f) {
            if (mu < 0 || mv < 0) continue;
            const int mx = (int)mu, my = (int)mv;
            if (mx >= m.w || my >= m.h || !bits[my * m.w + mx]) continue;
            if (clipR > 0 && px * px + py * py > clip2) continue;
            row[sx] = c;
        }
    }
}

static Mask LABEL[4];        // "10" "20" "30" "40" (index = tens - 1)
static Mask TXT_NODATA, TXT_OFF, TXT_DEMO;

void begin() {
    static const char* labels[4] = {"10", "20", "30", "40"};
    for (int i = 0; i < 4; i++) LABEL[i] = makeMask(labels[i], &FreeSansBold9pt7b);
    TXT_NODATA = makeMask("NO DATA", &FreeSansBold12pt7b);
    TXT_OFF    = makeMask("OFF", &FreeSansBold12pt7b);
    TXT_DEMO   = makeMask("DEMO", &FreeSansBold12pt7b);
    strip = (uint16_t*)heap_caps_malloc(STRIP_ROWS * LCD_W * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

// ------------------------------------------------------------------ layers
// 1. Sky and ground, row by row. A pixel is sky when its card v coordinate is above 0.
static IRAM_ATTR void drawSkyGround() {
    for (int sy = Y0; sy < Y1; sy++) {
        const float py = CY - (sy + 0.5f);
        const float hw = circleHalfWidth(R_CARD, py);
        if (hw < 0) { hspan(sy, 0, LCD_W - 1, C_BEZEL); continue; }
        const int xl = iceil(-hw + CX - 0.5f), xr = ifloor(hw + CX - 0.5f);
        hspan(sy, 0, xl - 1, C_BEZEL);
        hspan(sy, xr + 1, LCD_W - 1, C_BEZEL);

        // v(px) = c - s * px
        const float s = F.sb, c = py * F.cb + F.pitchPx;
        if (fabsf(s) < 1e-5f) { hspan(sy, xl, xr, c > 0 ? C_SKY : C_GROUND); continue; }
        int xs = ifloor(c / s + CX - 0.5f);        // last pixel left of the crossing
        if (xs < xl - 1) xs = xl - 1;
        if (xs > xr) xs = xr;
        const uint16_t left = (s > 0) ? C_SKY : C_GROUND;
        const uint16_t right = (s > 0) ? C_GROUND : C_SKY;
        hspan(sy, xl, xs, left);
        hspan(sy, xs + 1, xr, right);
    }
}

// 2. Horizon line and the perspective lines on the ground half.
static IRAM_ATTR void drawHorizonAndGround() {
    // Any visible chord of the horizon lies within R_CARD of the horizon centre.
    cardRect(-R_CARD, R_CARD, -1.6f, 1.6f, C_WHITE, R_CARD);
    // Lines radiating down from the horizon centre, as in the classic instrument.
    static const float angles[] = {-60, -30, 30, 60};
    for (float a : angles) {
        const float r = R_LADDER + 30 + fabsf(F.pitchPx);
        cardSegment(0, 0, r * sinf(a * DEG), -r * cosf(a * DEG), 1.2f, C_WHITE, R_LADDER + 30);
    }
}

// 3. Pitch ladder: short lines every 5 degrees, long numbered lines every 10.
static IRAM_ATTR void drawLadder(float pitchDeg) {
    const int lo = ifloor((pitchDeg - 30) / 5) * 5;
    const int hi = iceil((pitchDeg + 30) / 5) * 5;
    for (int d = lo; d <= hi; d += 5) {
        if (d == 0 || d < -90 || d > 90) continue;
        const float v = d * PPD;
        const bool major = (d % 10) == 0;
        const float half = major ? (abs(d) % 20 == 0 ? 50.0f : 38.0f) : 18.0f;
        cardRect(-half, half, v - 1.5f, v + 1.5f, C_WHITE, R_LADDER);
        if (major && abs(d) <= 40) {
            const Mask& m = LABEL[abs(d) / 10 - 1];
            const float off = half + 8 + m.w * 0.5f;
            for (float side : {-1.0f, 1.0f}) {
                Pt p = card(side * off, v);
                if (p.x * p.x + p.y * p.y < (R_LADDER - 8) * (R_LADDER - 8))
                    drawMask(m, p, F.bankRad, C_WHITE, R_LADDER);
            }
        }
    }
}


// 4. Bank scale on the rotating ring: marks at 10, 20, 30, 45, 60 and 90 degrees each side,
//    and a white index triangle at zero. Rotates with bank but does not move with pitch.
static IRAM_ATTR void ringSegment(float a, float r0, float r1, float halfWidth, uint16_t c) {
    const float s = sinf(a * DEG), co = cosf(a * DEG);
    const float tu = co * halfWidth, tv = -s * halfWidth;      // tangent offset
    Pt q[4] = {ring(r0 * s + tu, r0 * co + tv), ring(r1 * s + tu, r1 * co + tv),
               ring(r1 * s - tu, r1 * co - tv), ring(r0 * s - tu, r0 * co - tv)};
    fillConvex(q, 4, c, R_CARD);
}

static IRAM_ATTR void ringTriangle(float a, float rTip, float rBase, float halfBase, uint16_t c) {
    const float s = sinf(a * DEG), co = cosf(a * DEG);
    const float tu = co * halfBase, tv = -s * halfBase;
    Pt q[3] = {ring(rTip * s, rTip * co), ring(rBase * s + tu, rBase * co + tv),
               ring(rBase * s - tu, rBase * co - tv)};
    fillConvex(q, 3, c, R_CARD);
}

static IRAM_ATTR void drawBankScale() {
    static const float shortTicks[] = {10, 20};
    static const float longTicks[] = {30, 60};
    for (float side : {-1.0f, 1.0f}) {
        for (float a : shortTicks) ringSegment(side * a, 196, 210, 2.0f, C_WHITE);
        for (float a : longTicks) ringSegment(side * a, 190, 222, 2.5f, C_WHITE);
        ringSegment(side * 90, 186, 222, 3.0f, C_WHITE);
        ringTriangle(side * 45, 198, 214, 6.0f, C_WHITE);
    }
    ringTriangle(0, 194, 218, 12.0f, C_WHITE);                 // zero-bank index
}

// 5. Fixed parts: bank pointer, pedestal, miniature airplane, rim and status flag.
static IRAM_ATTR void fixedRect(float x0, float x1, float y0, float y1, uint16_t c, float clipR = 0) {
    Pt q[4] = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
    fillConvex(q, 4, c, clipR);
}

static IRAM_ATTR void drawFixedParts(int offset) {
    const float a = (float)offset;

    // Bank pointer at the top, pointing up at the scale.
    Pt pointer[3] = {{0, 190}, {-12, 168}, {12, 168}};
    fillConvex(pointer, 3, C_ORANGE, 0);

    // Pedestal holding the miniature airplane.
    Pt pedestal[4] = {{-9, a - 16}, {9, a - 16}, {46, -R_CARD - 10}, {-46, -R_CARD - 10}};
    fillConvex(pedestal, 4, C_PEDESTAL, R_CARD);

    // Miniature airplane: wings with turned-down inner ends, a centre post and dot.
    fixedRect(-104, -36, a - 3.5f, a + 3.5f, C_ORANGE);
    fixedRect(36, 104, a - 3.5f, a + 3.5f, C_ORANGE);
    fixedRect(-40, -33, a - 14, a + 3.5f, C_ORANGE);
    fixedRect(33, 40, a - 14, a + 3.5f, C_ORANGE);
    fixedRect(-2.5f, 2.5f, a - 16, a, C_ORANGE);
    Pt dot[8];
    for (int i = 0; i < 8; i++) dot[i] = {5.5f * cosf(i * 45 * DEG), a + 5.5f * sinf(i * 45 * DEG)};
    fillConvex(dot, 8, C_ORANGE, 0);
}

static IRAM_ATTR void drawRim() {
    // Thin dark ring where the card meets the bezel.
    for (int sy = Y0; sy < Y1; sy++) {
        const float py = CY - (sy + 0.5f);
        const float outer = circleHalfWidth(R_CARD + 4, py);
        if (outer < 0) continue;
        const float inner = circleHalfWidth(R_CARD, py);
        if (inner < 0) {
            hspan(sy, iceil(-outer + CX - 0.5f), ifloor(outer + CX - 0.5f), C_RIM);
        } else {
            hspan(sy, iceil(-outer + CX - 0.5f), ifloor(-inner + CX - 0.5f), C_RIM);
            hspan(sy, iceil(inner + CX - 0.5f), ifloor(outer + CX - 0.5f), C_RIM);
        }
    }
}

static IRAM_ATTR void drawFlag(Flag flag) {
    if (flag == Flag::None) return;
    const Mask& m = flag == Flag::Off ? TXT_OFF : flag == Flag::NoData ? TXT_NODATA : TXT_DEMO;
    const uint16_t box = flag == Flag::Demo ? C_BLACK : C_RED;
    const uint16_t ink = flag == Flag::Demo ? C_AMBER : C_WHITE;
    const float hw = m.w * 0.5f + 12, cy = 118;
    fixedRect(-hw, hw, cy - 17, cy + 17, box);
    drawMask(m, {0, cy}, 0, ink, 0);
}

// ------------------------------------------------------------------ frame
void draw(uint16_t* fb, const AttitudeView& v) {
    float pitch = v.pitchDeg;
    if (pitch > 90) pitch = 90;
    if (pitch < -90) pitch = -90;
    F.bankRad = v.bankDeg * DEG;
    F.cb = cosf(F.bankRad);
    F.sb = sinf(F.bankRad);
    F.pitchPx = pitch * PPD;

    // The bezel outside the rim never changes, so after both frame buffers hold a full
    // frame only the inner disc is copied.
    static int staticFrames = 0;
    for (Y0 = 0; Y0 < LCD_H; Y0 += STRIP_ROWS) {
        Y1 = min(Y0 + STRIP_ROWS, LCD_H);
        FB = strip - Y0 * LCD_W;
        drawSkyGround();
        drawHorizonAndGround();
        drawLadder(pitch);
        drawBankScale();
        drawRim();
        drawFixedParts(v.airplaneOffset);
        drawFlag(v.flag);
        if (staticFrames < 2) {
            memcpy(fb + Y0 * LCD_W, strip, (Y1 - Y0) * LCD_W * 2);
        } else {
            for (int sy = Y0; sy < Y1; sy++) {
                const float hw = circleHalfWidth(R_CARD + 5, CY - (sy + 0.5f));
                if (hw < 0) continue;
                const int x0 = max(0, ifloor(CX - hw)), x1 = min(LCD_W - 1, iceil(CX + hw));
                memcpy(fb + sy * LCD_W + x0, strip + (sy - Y0) * LCD_W + x0, (x1 - x0 + 1) * 2);
            }
        }
    }
    if (staticFrames < 2) staticFrames++;
}

}  // namespace renderer
