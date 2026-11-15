#include "display.h"
#include "board.h"
#include "expander.h"
#include <Arduino.h>
#include <driver/spi_master.h>
#include <esp_idf_version.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <freertos/semphr.h>

namespace display {

static esp_lcd_panel_handle_t panel = nullptr;
static uint16_t* fb[2] = {nullptr, nullptr};
static int back = 1;
static SemaphoreHandle_t vsyncSem = nullptr;

// ---------------------------------------------------------------- ST7701 configuration
// Register sequence for this board's 2.1" round panel (TL021WVC02-class), taken from the
// factory firmware shipped on the board. Note: the sequence in Waveshare's current demo
// download is for a different panel batch; on this board it leaves the image rolling and
// can leave the panel washed out for a while afterwards.
// Each entry is a command followed by its parameters, sent over a 3-wire 9-bit SPI link
// (first bit: 0 = command, 1 = data).
struct InitCmd { uint8_t cmd; uint8_t len; uint8_t data[16]; uint16_t delayMs; };
static const InitCmd ST7701_INIT[] = {
    {0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x13}, 0},
    {0xEF, 1, {0x08}, 0},
    {0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x10}, 0},
    {0xC0, 2, {0x3B, 0x00}, 0},
    {0xC1, 2, {0x10, 0x0C}, 0},
    {0xC2, 2, {0x07, 0x0A}, 0},
    {0xC7, 1, {0x00}, 0},
    {0xCC, 1, {0x10}, 0},
    {0xCD, 1, {0x08}, 0},
    {0xB0, 16, {0x05, 0x12, 0x98, 0x0E, 0x0F, 0x07, 0x07, 0x09, 0x09, 0x23, 0x05, 0x52, 0x0F, 0x67, 0x2C, 0x11}, 0},
    {0xB1, 16, {0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08, 0x08, 0x22, 0x03, 0x51, 0x11, 0x66, 0x2B, 0x0F}, 0},
    {0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x11}, 0},
    {0xB0, 1, {0x5D}, 0},
    {0xB1, 1, {0x3E}, 0},
    {0xB2, 1, {0x81}, 0},
    {0xB3, 1, {0x80}, 0},
    {0xB5, 1, {0x4E}, 0},
    {0xB7, 1, {0x85}, 0},
    {0xB8, 1, {0x20}, 0},
    {0xC1, 1, {0x78}, 0},
    {0xC2, 1, {0x78}, 0},
    {0xD0, 1, {0x88}, 0},
    {0xE0, 3, {0x00, 0x00, 0x02}, 0},
    {0xE1, 11, {0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30, 0x00, 0x33, 0x33}, 0},
    {0xE2, 12, {0x11, 0x11, 0x33, 0x33, 0xF4, 0x00, 0x00, 0x00, 0xF4, 0x00, 0x00, 0x00}, 0},
    {0xE3, 4, {0x00, 0x00, 0x11, 0x11}, 0},
    {0xE4, 2, {0x44, 0x44}, 0},
    {0xE5, 16, {0x0D, 0xF5, 0x30, 0xF0, 0x0F, 0xF7, 0x30, 0xF0, 0x09, 0xF1, 0x30, 0xF0, 0x0B, 0xF3, 0x30, 0xF0}, 0},
    {0xE6, 4, {0x00, 0x00, 0x11, 0x11}, 0},
    {0xE7, 2, {0x44, 0x44}, 0},
    {0xE8, 16, {0x0C, 0xF4, 0x30, 0xF0, 0x0E, 0xF6, 0x30, 0xF0, 0x08, 0xF0, 0x30, 0xF0, 0x0A, 0xF2, 0x30, 0xF0}, 0},
    {0xE9, 2, {0x36, 0x01}, 0},
    {0xEB, 7, {0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40}, 0},
    {0xED, 16, {0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF, 0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF}, 0},
    {0xEF, 6, {0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}, 0},
    {0xFF, 5, {0x77, 0x01, 0x00, 0x00, 0x00}, 0},
    {0x11, 0, {0}, 120},       // sleep out
    {0x3A, 1, {0x66}, 0},      // pixel format
    {0x36, 1, {0x00}, 0},      // scan direction
    {0x35, 1, {0x00}, 0},      // tearing effect line on
    {0x29, 0, {0}, 0},         // display on
};

static spi_device_handle_t spi = nullptr;

static int spiErrors = 0;
static void spiWrite9(bool isData, uint8_t value) {
    spi_transaction_t t = {};
    t.cmd = isData ? 1 : 0;   // 1-bit "command" phase carries the D/C bit
    t.addr = value;           // 8-bit "address" phase carries the byte
    if (spi_device_polling_transmit(spi, &t) != ESP_OK) spiErrors++;
}

static void configurePanel() {
    spi_bus_config_t bus = {};
    bus.mosi_io_num = PIN_LCD_SPI_SDA;
    bus.miso_io_num = -1;
    bus.sclk_io_num = PIN_LCD_SPI_SCL;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 64;
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED);
    Serial.printf("panel: spi_bus_initialize %s\n", esp_err_to_name(e));

    spi_device_interface_config_t dev = {};
    dev.command_bits = 1;
    dev.address_bits = 8;
    dev.mode = 0;
    dev.clock_speed_hz = 10 * 1000 * 1000;
    dev.spics_io_num = -1;          // chip select is on the expander
    dev.queue_size = 1;
    e = spi_bus_add_device(SPI2_HOST, &dev, &spi);
    Serial.printf("panel: spi_bus_add_device %s\n", esp_err_to_name(e));

    // Hardware reset, then select the panel for the whole sequence.
    expander::write(EXIO_LCD_RESET, false);
    delay(10);
    expander::write(EXIO_LCD_RESET, true);
    delay(50);
    expander::write(EXIO_LCD_CS, false);
    delay(5);
    for (const InitCmd& c : ST7701_INIT) {
        spiWrite9(false, c.cmd);
        for (int i = 0; i < c.len; i++) spiWrite9(true, c.data[i]);
        if (c.delayMs) delay(c.delayMs);
    }
    expander::write(EXIO_LCD_CS, true);
    Serial.printf("panel: init sequence sent, %d SPI errors\n", spiErrors);

    // GPIO1/2 are shared with the SD card slot; release the bus now that the panel is set up.
    spi_bus_remove_device(spi);
    spi_bus_free(SPI2_HOST);
}

// ---------------------------------------------------------------- RGB interface
static bool IRAM_ATTR onVsync(esp_lcd_panel_handle_t, const esp_lcd_rgb_panel_event_data_t*, void*) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(vsyncSem, &woken);
    return woken == pdTRUE;
}

static bool startRgb() {
    esp_lcd_rgb_panel_config_t cfg = {};
    cfg.clk_src = LCD_CLK_SRC_DEFAULT;
    cfg.timings.pclk_hz = 16 * 1000 * 1000;   // ~58 Hz refresh
    cfg.timings.h_res = LCD_W;
    cfg.timings.v_res = LCD_H;
    cfg.timings.hsync_pulse_width = 8;
    cfg.timings.hsync_back_porch = 10;
    cfg.timings.hsync_front_porch = 50;
    cfg.timings.vsync_pulse_width = 3;
    cfg.timings.vsync_back_porch = 8;
    cfg.timings.vsync_front_porch = 8;
    cfg.data_width = 16;
    cfg.bits_per_pixel = 16;
    cfg.num_fbs = 2;
    // Bounce buffers copy each frame out of PSRAM through internal RAM, which stops the
    // image drifting when PSRAM is busy.
    cfg.bounce_buffer_size_px = 10 * LCD_W;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    cfg.dma_burst_size = 64;
#else
    cfg.psram_trans_align = 64;
#endif
    cfg.hsync_gpio_num = PIN_LCD_HSYNC;
    cfg.vsync_gpio_num = PIN_LCD_VSYNC;
    cfg.de_gpio_num = PIN_LCD_DE;
    cfg.pclk_gpio_num = PIN_LCD_PCLK;
    cfg.disp_gpio_num = -1;
    for (int i = 0; i < 16; i++) cfg.data_gpio_nums[i] = PIN_LCD_DATA[i];
    cfg.flags.fb_in_psram = 1;

    esp_err_t e = esp_lcd_new_rgb_panel(&cfg, &panel);
    Serial.printf("panel: esp_lcd_new_rgb_panel %s\n", esp_err_to_name(e));
    if (e != ESP_OK) return false;
    vsyncSem = xSemaphoreCreateBinary();
    esp_lcd_rgb_panel_event_callbacks_t cbs = {};
    cbs.on_vsync = onVsync;
    esp_lcd_rgb_panel_register_event_callbacks(panel, &cbs, nullptr);
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    void* a = nullptr;
    void* b = nullptr;
    esp_lcd_rgb_panel_get_frame_buffer(panel, 2, &a, &b);
    fb[0] = static_cast<uint16_t*>(a);
    fb[1] = static_cast<uint16_t*>(b);
    Serial.printf("panel: frame buffers %p %p\n", a, b);
    memset(fb[0], 0, LCD_W * LCD_H * 2);
    memset(fb[1], 0, LCD_W * LCD_H * 2);
    back = 1;
    return fb[0] && fb[1];
}

// ---------------------------------------------------------------- backlight
void setBacklight(uint8_t percent) {
    if (percent > 100) percent = 100;
    ledcWrite(PIN_LCD_BACKLIGHT, (uint32_t)percent * 1023 / 100);
}

bool begin() {
    ledcAttach(PIN_LCD_BACKLIGHT, 20000, 10);
    setBacklight(0);                 // stay dark until the first frame is drawn
    configurePanel();
    return startRgb();
}

uint16_t* backBuffer() { return fb[back]; }

void present() {
    // Passing one of the panel's own frame buffers makes the driver switch to it at the
    // next frame instead of copying. Wait for that refresh before drawing into the
    // buffer that has just been retired.
    esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_W, LCD_H, fb[back]);
    xSemaphoreTake(vsyncSem, 0);                       // drop a stale signal
    xSemaphoreTake(vsyncSem, pdMS_TO_TICKS(40));
    back ^= 1;
}

}  // namespace display
