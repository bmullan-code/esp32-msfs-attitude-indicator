// Pin map for the Waveshare ESP32-S3-Touch-LCD-2.1.
// Source: Waveshare's wiki and LVGL_Arduino demo for this board.
#pragma once

// Shared I2C bus: touch (0x15), TCA9554 expander (0x20), RTC (0x51), IMU (0x6B)
constexpr int PIN_I2C_SDA = 15;
constexpr int PIN_I2C_SCL = 7;

// ST7701 configuration interface (3-wire SPI, 9-bit). CS and RESET sit on the expander.
constexpr int PIN_LCD_SPI_SDA = 1;
constexpr int PIN_LCD_SPI_SCL = 2;

// RGB interface
constexpr int PIN_LCD_HSYNC = 38;
constexpr int PIN_LCD_VSYNC = 39;
constexpr int PIN_LCD_DE    = 40;
constexpr int PIN_LCD_PCLK  = 41;
// RGB565 data lines, D0 (blue LSB) .. D15 (red MSB)
constexpr int PIN_LCD_DATA[16] = {5, 45, 48, 47, 21, 14, 13, 12, 11, 10, 9, 46, 3, 8, 18, 17};

constexpr int PIN_LCD_BACKLIGHT = 6;
constexpr int PIN_TOUCH_INT     = 16;
constexpr int PIN_BOOT_BUTTON   = 0;

// TCA9554 expander outputs (1-based EXIO numbers as printed in Waveshare's docs)
constexpr int EXIO_LCD_RESET   = 1;
constexpr int EXIO_TOUCH_RESET = 2;
constexpr int EXIO_LCD_CS      = 3;
constexpr int EXIO_SD_CS       = 4;
constexpr int EXIO_BUZZER      = 8;   // HIGH = buzzer on

constexpr int LCD_W = 480;
constexpr int LCD_H = 480;
