/*
 * JUNKTENDO — bring-up 02: LSM6DSOX over I2C, data on the GMT130 (ST7789)
 * Target: ESP32-DevKitC v4 (WROOM-32U)
 *
 * Wiring:
 *   LSM6DSOX (STEMMA QT):
 *     VIN -> 3V3, GND -> GND
 *     SDA -> GPIO21, SCL -> GPIO22
 *   GMT130 1.3" 240x240 IPS (ST7789, SPI):
 *     VCC -> 3V3   * strictly 3.3 V — this module has no regulator *
 *     GND -> GND
 *     SCL -> GPIO18 (SPI SCK)   * SDA/SCL here are SPI, not I2C *
 *     SDA -> GPIO23 (SPI MOSI)
 *     RES -> GPIO4
 *     DC  -> GPIO2
 *     BLK -> 3V3   (later: GPIO + PWM for the brightness control)
 *     If your GMT130 has a CS pin: wire it to GPIO5 and set TFT_CS to 5
 *     in the TFT_eSPI User_Setup (see the Junktendo TFT setup canvas).
 *
 * Needs TFT_eSPI (Bodmer) with the User_Setup from the sibling canvas.
 *
 * Expected: JUNKTENDO header, live acc/gyr values at 10 Hz, and a tilt
 * dot that slides around as you lean the board — d-pad proof of life.
 */

#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// ---------- LSM6DSOX registers (raw, no library) ----------
static const uint8_t LSM_ADDR      = 0x6A;  // 0x6B if you solder the A0 jumper
static const uint8_t REG_WHO_AM_I  = 0x0F;  // expect 0x6C
static const uint8_t REG_CTRL1_XL  = 0x10;  // accel config
static const uint8_t REG_CTRL2_G   = 0x11;  // gyro config
static const uint8_t REG_CTRL3_C   = 0x12;  // control/misc
static const uint8_t REG_OUTX_L_G  = 0x22;  // gyro X low .. accel Z high, 12 bytes
static const uint8_t WHO_AM_I_CODE = 0x6C;

// Sensitivities for our chosen ranges: ±2 g and 245 dps
static const float ACCEL_LSB_G     = 0.000061f;
static const float GYRO_LSB_DPS    = 0.00875f;

static const int PIN_SDA = 21;
static const int PIN_SCL = 22;

// ---------- tiny I2C helpers ----------
bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;  // repeated start
  int got = Wire.requestFrom((int)addr, (int)n);
  for (int i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}

// ---------- screen helpers ----------
// Overwrite a size-2 text line without flicker (blank the region first).
void printVal(int x, int y, uint16_t color, const char *label, float v) {
  char buf[20];
  snprintf(buf, sizeof(buf), "%s%+8.2f", label, v);  // fixed width absorbs sign changes
  tft.fillRect(x, y, 156, 16, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print(buf);
}

// Tilt dot: acc X/Y gravity vector shown inside the little box.
void drawTilt(float axg, float ayg) {
  tft.fillRect(170, 50, 62, 62, TFT_BLACK);
  tft.drawRect(170, 50, 62, 62, TFT_DARKGREY);
  tft.drawFastHLine(175, 81, 52, TFT_DARKGREY);
  tft.drawFastVLine(201, 55, 52, TFT_DARKGREY);
  int dx = constrain((int)(axg * 28.0f), -28, 28);
  int dy = constrain((int)(ayg * 28.0f), -28, 28);
  tft.fillCircle(201 + dx, 81 - dy, 5, TFT_GREENYELLOW);
}

void drawHeader() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_SILVER, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(12, 12);
  tft.print("JUNKTENDO");
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(14, 32);
  tft.print("bring-up 02: IMU -> GMT130");

  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(8, 52);
  tft.print("ACC [g]");
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setCursor(8, 152);
  tft.print("GYR [dps]");
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nJUNKTENDO bring-up 02: IMU -> GMT130\n");

  // Screen first, so errors are visible even if the IMU is dead
  tft.init();  // no size args in TFT_eSPI — 240x240 comes from User_Setup
  tft.setRotation(0);
  drawHeader();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);  // verified in bring-up 01, now full speed

  // Identify
  uint8_t id = 0;
  if (i2cRead(LSM_ADDR, REG_WHO_AM_I, &id, 1) != 1) {
    Serial.println("LSM6DSOX not ACKing at 0x6A — wiring/address problem, halting.");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(8, 100);
    tft.print(" IMU absent!");
    while (true) delay(1000);
  }
  Serial.printf("WHO_AM_I = 0x%02X (expect 0x%02X)\n", id, WHO_AM_I_CODE);
  if (id != WHO_AM_I_CODE) {
    Serial.println("Wrong WHO_AM_I! Verify part and ADDR jumper state.");
    while (true) delay(1000);
  }

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(14, 42);
  tft.printf("I2C 0x%02X  WHO_AM_I 0x%02X", LSM_ADDR, id);

  // Soft reset, keeping auto-increment enabled
  i2cWrite(LSM_ADDR, REG_CTRL3_C, 0x81);  // SW_RESET | IF_INC
  delay(10);

  // Configure: BDU + auto-increment, accel 104 Hz ±2 g, gyro 104 Hz 245 dps
  i2cWrite(LSM_ADDR, REG_CTRL3_C, 0x44);   // BDU | IF_INC
  i2cWrite(LSM_ADDR, REG_CTRL1_XL, 0x40);  // ODR 104 Hz, FS ±2 g
  i2cWrite(LSM_ADDR, REG_CTRL2_G, 0x40);   // ODR 104 Hz, FS 245 dps
  delay(100);
}

void loop() {
  uint8_t raw[12];
  if (i2cRead(LSM_ADDR, REG_OUTX_L_G, raw, 12) != 12) {
    Serial.println("read failed");
    delay(100);
    return;
  }

  int16_t gx = (int16_t)((raw[1] << 8) | raw[0]);
  int16_t gy = (int16_t)((raw[3] << 8) | raw[2]);
  int16_t gz = (int16_t)((raw[5] << 8) | raw[4]);
  int16_t ax = (int16_t)((raw[7] << 8) | raw[6]);
  int16_t ay = (int16_t)((raw[9] << 8) | raw[8]);
  int16_t az = (int16_t)((raw[11] << 8) | raw[10]);

  float axg = ax * ACCEL_LSB_G, ayg = ay * ACCEL_LSB_G, azg = az * ACCEL_LSB_G;
  float gxd = gx * GYRO_LSB_DPS, gyd = gy * GYRO_LSB_DPS, gzd = gz * GYRO_LSB_DPS;

  // Screen (10 Hz refresh, flicker-free by design)
  printVal(8, 70,  TFT_GREENYELLOW, "X", axg);
  printVal(8, 90,  TFT_GREENYELLOW, "Y", ayg);
  printVal(8, 110, TFT_GREENYELLOW, "Z", azg);
  drawTilt(axg, ayg);
  printVal(8, 170, TFT_ORANGE, "X", gxd);
  printVal(8, 190, TFT_ORANGE, "Y", gyd);
  printVal(8, 210, TFT_ORANGE, "Z", gzd);

  // Serial mirror (115200)
  Serial.printf("acc[g] x=%6.2f y=%6.2f z=%6.2f | gyr[dps] x=%7.1f y=%7.1f z=%7.1f\n",
                axg, ayg, azg, gxd, gyd, gzd);

  delay(100);  // ~10 Hz UI; sensor samples at 104 Hz internally
}