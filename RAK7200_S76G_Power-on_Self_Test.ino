/*
 * POST_S76G(.ino) firmware
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * Author: Linar Yusupov, linar.r.yusupov@gmail.com
 *
 * Web: http://github.com/lyusupov/SoftRF
 *
 * Credits:
 *   Arduino Core for STM32 is developed by Frederic Pillon
 *   U8g2 monochrome LCD, OLED and eInk library is developed by Oliver Kraus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Wire.h>
#include <SPI.h>
#include <U8x8lib.h>

#define SX1276_RegVersion     0x42
#define SSD1306_OLED_I2C_ADDR 0x3C
#define BMP085_I2CADDR        0x77
#define BMP280_ADDRESS        0x77
#define BMP280_ADDRESS_ALT    0x76 /* GY-91, SA0 is NC */

#define BMP280_REGISTER_CHIPID 0xD0

#define BMP280_CHIPID         (0x58)
#define BME280_CHIPID         (0x60)

#if !defined(USBD_USE_CDC) || defined(DISABLE_GENERIC_SERIALUSB)
#define Serial                Serial1
#endif

// AcSiP S7xx UART1 (Console)
#define S7xx_CONSOLE_TX                   PA9  // UART1 (CH340E U1)
#define S7xx_CONSOLE_RX                   PA10 // UART1 (CH340E U1)

// AcSiP S7xx Internal SPI2 STM32L073RZ(U|Y)x <--> SX127x
#define S7xx_SX127x_MOSI                  PB15 // SPI2
#define S7xx_SX127x_MISO                  PB14 // SPI2
#define S7xx_SX127x_SCK                   PB13 // SPI2
#define S7xx_SX127x_NSS                   PB12 // SPI2
#define S7xx_SX127x_NRESET                PB10
#define S7xx_SX127x_DIO0                  PB11
#define S7xx_SX127x_DIO1                  PC13
#define S7xx_SX127x_DIO2                  PB9
#define S7xx_SX127x_DIO3                  PB4  // unused
#define S7xx_SX127x_DIO4                  PB3  // unused
#define S7xx_SX127x_DIO5                  PA15 // unused
#define S7xx_SX127x_ANTENNA_SWITCH_RXTX   PA1  // Radio Antenna Switch 1:RX, 0:TX

// AcSiP S7xG SONY CXD5603GF GNSS
#define RAK7200_S76G_CXD5603_POWER_ENABLE PC4  // Enable 1V8 Power to GNSS (U2 TPS62740)
#define T_Motion_S76G_CXD5603_1PPS        PB5  // TTGO T-Motion 1PPS
#define S7xG_CXD5603_RESET                PB2  // Reset does not appear to work
#define S7xG_CXD5603_LEVEL_SHIFTER        PC6
#define S7xG_CXD5603_UART_TX              PC10 // UART4
#define S7xG_CXD5603_UART_RX              PC11 // UART4
#define S7xG_CXD5603_BAUD_RATE            115200

// AcSiP S7xx I2C1
#define S7xx_I2C_SCL                      PB6  // I2C1
#define S7xx_I2C_SDA                      PB7  // I2C1

HardwareSerial Serial1(S7xx_CONSOLE_RX, S7xx_CONSOLE_TX);
HardwareSerial Serial3(S7xG_CXD5603_UART_RX, S7xG_CXD5603_UART_TX);

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8_i2c(U8X8_PIN_NONE);

static U8X8_SSD1306_128X64_NONAME_HW_I2C *u8x8 = NULL;

static bool GNSS_probe() {

    unsigned long startTime = millis();
    char c1, c2;
    c1 = c2 = 0;

    // clean any leftovers
    Serial3.flush();

    // Timeout if no valid response in 3 seconds
    while (millis() - startTime < 3000) {

        if (Serial3.available() > 0) {
            c1 = Serial3.read();
            if ((c1 == '$') && (c2 == 0)) {
                c2 = c1;
                continue;
            }
            if ((c2 == '$') && (c1 == 'G')) {
                /* got $G */

                /* leave the function with GNSS port opened */
                return true;
            }
            else {
                c2 = 0;
            }
        }

        delay(1);
    }

    return false;
}

static bool bmp_probe() {
#if 1

    Wire.beginTransmission(BMP280_ADDRESS);
    if (Wire.endTransmission() == 0) return true;
    Wire.beginTransmission(BMP280_ADDRESS_ALT);
    if (Wire.endTransmission() == 0) return true;

#else

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REGISTER_CHIPID);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, (byte)1);
    if (Wire.read() == BMP280_CHIPID) return true;

    Wire.beginTransmission(BMP280_ADDRESS_ALT);
    Wire.write(BMP280_REGISTER_CHIPID);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS_ALT, (byte)1);
    if (Wire.read() == BMP280_CHIPID) return true;

    Wire.beginTransmission(BMP280_ADDRESS);
    Wire.write(BMP280_REGISTER_CHIPID);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS, (byte)1);
    if (Wire.read() == BME280_CHIPID) return true;

    Wire.beginTransmission(BMP280_ADDRESS_ALT);
    Wire.write(BMP280_REGISTER_CHIPID);
    Wire.endTransmission();
    Wire.requestFrom(BMP280_ADDRESS_ALT, (byte)1);
    if (Wire.read() == BME280_CHIPID) return true;

#endif

    return false;
}

static void scanI2Cbus(void) {
    byte err, addr;
    int nDevices = 0;

    Serial.println("Scanning I2C bus");
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            Serial.print(addr < 16 ? "0" : "");
            //if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            nDevices++;
        }
        else
            if (err == 4) {
                Serial.print("Unknow error at address 0x");
                Serial.print(addr < 16 ? "0" : "");
                //if (addr < 16) Serial.print("0");
                Serial.println(addr, HEX);
            }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("Scanning complete\n");
}

static void SerialPassThrough(void) {
    if (Serial.available()) {
        Serial3.write(Serial.read());
    }
    if (Serial3.available()) {
        Serial.write(Serial3.read());
    }
}

void setup() {

    bool has_SX1276 = false;
    bool has_GNSS = false;
    bool has_OLED = false;
    bool has_BMP280 = false;

    Serial.begin(115200);

#if defined(USBD_USE_CDC) && !defined(DISABLE_GENERIC_SERIALUSB)
    /* Let host's USB and console drivers to warm-up */
    delay(2000);
#else
    delay(500);
#endif

    // Power On RAK7200 S7xG GNSS
    pinMode(RAK7200_S76G_CXD5603_POWER_ENABLE, OUTPUT);
    digitalWrite(RAK7200_S76G_CXD5603_POWER_ENABLE, HIGH);
    delay(1250); // Delay 315µs to 800µs ramp up time

    delay(2000);
    Serial.println();
    Serial.println(F("RAK7200 (S76G) Power-on Self Test"));
    Serial.println();
    Serial.flush();

    Wire.setSCL(S7xx_I2C_SCL);
    Wire.setSDA(S7xx_I2C_SDA);
    Wire.begin();

    /* SSD1306 I2C OLED probing */
    Wire.beginTransmission(SSD1306_OLED_I2C_ADDR);
    has_OLED = (Wire.endTransmission() == 0 ? true : false);

    if (has_OLED) {
        u8x8 = &u8x8_i2c;
    }

    if (u8x8) {
        u8x8->begin();
        u8x8->setFont(u8x8_font_chroma48medium8_r);
        u8x8->clear();
        u8x8->draw2x2String(1, 1, "RAK7200");
        u8x8->drawString(6, 4, "");
        u8x8->draw2x2String(2, 6, "");
        delay(3000);
    }

    SPI.setMISO(S7xx_SX127x_MISO);
    SPI.setMOSI(S7xx_SX127x_MOSI);
    SPI.setSCLK(S7xx_SX127x_SCK);
    SPI.setSSEL(S7xx_SX127x_NSS);

    SPI.begin();

    digitalWrite(S7xx_SX127x_NSS, HIGH);
    pinMode(S7xx_SX127x_NSS, OUTPUT);

    digitalWrite(S7xx_SX127x_NRESET, HIGH);
    pinMode(S7xx_SX127x_NRESET, OUTPUT);

    // manually reset radio
    digitalWrite(S7xx_SX127x_NRESET, LOW); // drive RST pin low
    delay(5);
    digitalWrite(S7xx_SX127x_NRESET, HIGH); // drive RST pin high
    delay(5);

    digitalWrite(S7xx_SX127x_NSS, LOW);

    SPI.transfer(SX1276_RegVersion & 0x7F);
    has_SX1276 = (SPI.transfer(0x00) == 0x12 ? true : false);

    digitalWrite(S7xx_SX127x_NSS, HIGH);

    SPI.end();
    pinMode(S7xx_SX127x_NSS, INPUT);
    pinMode(S7xx_SX127x_NRESET, INPUT);

    Serial.println(F("Built-in components:"));

    Serial.print(F("RADIO  - "));
    Serial.println(has_SX1276 ? F("PASS") : F("FAIL"));

    if (u8x8) {
        u8x8->clear();
        u8x8->draw2x2String(0, 0, "RADIO");
        u8x8->draw2x2String(14, 0, has_SX1276 ? "+" : "-");
    }

    Serial3.begin(S7xG_CXD5603_BAUD_RATE);

    /* drive GNSS RST pin low */
    pinMode(S7xG_CXD5603_RESET, OUTPUT);
    digitalWrite(S7xG_CXD5603_RESET, LOW);

    /* activate 1.8V<->3.3V level shifters */
    pinMode(S7xG_CXD5603_LEVEL_SHIFTER, OUTPUT);
    digitalWrite(S7xG_CXD5603_LEVEL_SHIFTER, HIGH);

    /* keep RST low to ensure proper IC reset */
    delay(200);

    /* release */
    digitalWrite(S7xG_CXD5603_RESET, HIGH);

    /* give Sony GNSS few ms to warm up */
    delay(100);

    /* hot start */
    Serial3.write("@GSR\r\n");
    delay(250);

    has_GNSS = GNSS_probe();
    Serial.print(F("GNSS   - "));
    Serial.println(has_GNSS ? F("PASS") : F("FAIL"));

    if (u8x8) {
        u8x8->draw2x2String(0, 2, "GNSS");
        u8x8->draw2x2String(14, 2, has_GNSS ? "+" : "-");
    }

    Serial.println();
    Serial.println(F("External components:"));
    Serial.print(F("OLED   - "));
    Serial.println(has_OLED ? F("PASS") : F("FAIL"));

    if (u8x8) {
        u8x8->draw2x2String(0, 4, "OLED");
        u8x8->draw2x2String(14, 4, has_OLED ? "+" : "-");
    }

    has_BMP280 = bmp_probe();
    Serial.print(F("BMx280 - "));
    Serial.println(has_BMP280 ? F("PASS") : F("FAIL"));

    if (u8x8) {
        u8x8->draw2x2String(0, 6, "BMx280");
        u8x8->draw2x2String(14, 6, has_BMP280 ? "+" : "-");
    }

    Wire.end();

    Serial.println();
    Serial.println();
    Wire.setSCL(S7xx_I2C_SCL);
    Wire.setSDA(S7xx_I2C_SDA);
    Wire.begin();
    scanI2Cbus();
    Wire.end();

    Serial.println();
    Serial.println(F("POST is completed."));
    Serial.println();
    Serial.println();
    Serial.println();
}

void loop() {
  SerialPassThrough();
}
