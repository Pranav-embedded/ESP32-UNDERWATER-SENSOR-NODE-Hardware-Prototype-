#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <BMP280_DEV.h>
#include <MPU6050.h>
#include <PubSubClient.h>

// ── WiFi credentials ──────────────────────────────────────────
const char* WIFI_SSID     = "Beastpr";
const char* WIFI_PASSWORD = "Pranav007";

// ── MQTT broker  ───────────────

const char* MQTT_BROKER   = "10.181.151.107";  
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "esp32cam_node1";

// MQTT topics
const char* TOPIC_BMP     = "sensors/node1/bmp280";
const char* TOPIC_MPU     = "sensors/node1/mpu6050";
const char* TOPIC_STATUS  = "sensors/node1/status";
const char* TOPIC_CAM     = "sensors/node1/camera";

// ── Camera pins (AI-Thinker) ──────────────────────────────────
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ── I2C pins ──────────────────────────────────────────────────
#define I2C_SDA  15
#define I2C_SCL  14

// ── LED ───────────────────────────────────────────────────────
#define LED_PIN   4

// ── MPU6050 I2C address ───────────────────────────────────────
// AD0 pin → GND  = 0x68  (default, most common)
// AD0 pin → VCC  = 0x69
#define MPU_ADDR  0x68

// ─────────────────────────────────────────────────────────────
WebServer  server(80);
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
BMP280_DEV bmp280;
MPU6050    mpu(MPU_ADDR);   // pass address explicitly

struct SensorData {
  float temperature = 0, pressure = 0, altitude = 0;
  float accelX = 0, accelY = 0, accelZ = 0;
  float gyroX  = 0, gyroY  = 0, gyroZ  = 0;
  float roll   = 0, pitch  = 0;
  bool  bmpOK = false, mpuOK = false, sdOK = false, camOK = false;
};
SensorData sData;

// ═════════════════════════════════════════════════════════════
//  I2C RAW SCANNER
// ═════════════════════════════════════════════════════════════
void i2cScan() {
  Serial.println("\n[I2C] Scanning SDA=15 SCL=14 ...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("[I2C]  ✓ Device at 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("[I2C]  ✗ No devices! Check wiring & 3.3V supply.");
  Serial.println();
}

// ── Raw register read helper ──────────────────────────────────
uint8_t i2cReadReg(uint8_t devAddr, uint8_t regAddr) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;   // error sentinel
}

// ── Raw register write helper ─────────────────────────────────
bool i2cWriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t value) {
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

// ═════════════════════════════════════════════════════════════
//  MPU6050 MANUAL RESET & WAKE
//  If the Electronic Cats library's initialize() doesn't work,
//  we directly write the power-management register via Wire.
//  Reg 0x6B = PWR_MGMT_1: write 0x00 = wake up from sleep
//  Reg 0x6B: write 0x80 = full reset, then 0x00 to wake
// ═════════════════════════════════════════════════════════════
bool wakeupMPU(uint8_t addr) {
  // Step 1: device reset
  Serial.printf("[MPU] Sending reset to 0x%02X ...\n", addr);
  i2cWriteReg(addr, 0x6B, 0x80);  // PWR_MGMT_1: DEVICE_RESET bit
  delay(200);

  // Step 2: clear sleep bit (wake up)
  i2cWriteReg(addr, 0x6B, 0x00);  // wake, internal 8 MHz osc
  delay(100);

  // Step 3: verify WHO_AM_I register (0x75) returns 0x68
  uint8_t whoami = i2cReadReg(addr, 0x75);
  Serial.printf("[MPU] WHO_AM_I = 0x%02X (expect 0x68 or 0x72)\n", whoami);

  if (whoami == 0x68 || whoami == 0x72 || whoami == 0x70) {
    Serial.println("[MPU] WHO_AM_I OK");
    return true;
  }
  Serial.printf("[MPU] WHO_AM_I unexpected: 0x%02X\n", whoami);
  return false;
}

// ═════════════════════════════════════════════════════════════
//  MPU6050 INIT
// ═════════════════════════════════════════════════════════════
bool initMPU6050() {
  Serial.println("[MPU] Starting init...");

  // ── Step 1: verify device responds on I2C ────────────────
  Wire.beginTransmission(MPU_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("[MPU] No ACK at 0x%02X (err=%d) – check AD0 pin & wiring\n",
                  MPU_ADDR, err);
    // Try alternate address 0x69 (AD0 → VCC)
    Wire.beginTransmission(0x69);
    err = Wire.endTransmission();
    if (err == 0) {
      Serial.println("[MPU] Found at 0x69 instead! AD0 pin is HIGH (connected to VCC).");
      Serial.println("[MPU] Either pull AD0 to GND, or change MPU_ADDR to 0x69 in code.");
    }
    return false;
  }
  Serial.printf("[MPU] ACK received at 0x%02X\n", MPU_ADDR);

  // ── Step 2: manual reset + wake via raw I2C ──────────────
  if (!wakeupMPU(MPU_ADDR)) {
    Serial.println("[MPU] WHO_AM_I failed – sensor may be damaged or wired wrong");
    return false;
  }

  // ── Step 3: init via Electronic Cats library ─────────────
  mpu.initialize();
  delay(200);

  // ── Step 4: explicitly disable sleep (belt + suspenders) ─
  // PWR_MGMT_1 register 0x6B: set to 0x01 = PLL with X gyro
  i2cWriteReg(MPU_ADDR, 0x6B, 0x01);
  delay(100);

  // ── Step 5: set full-scale ranges explicitly ──────────────
  // ACCEL_CONFIG (0x1C): bits[4:3]=00 → ±2g
  i2cWriteReg(MPU_ADDR, 0x1C, 0x00);
  // GYRO_CONFIG  (0x1B): bits[4:3]=00 → ±250°/s
  i2cWriteReg(MPU_ADDR, 0x1B, 0x00);
  delay(50);

  // ── Step 6: testConnection via library ───────────────────
  if (!mpu.testConnection()) {
    Serial.println("[MPU] library testConnection() failed – but WHO_AM_I passed.");
    Serial.println("[MPU] Continuing anyway with raw reads...");
    // Return true here because WHO_AM_I already confirmed the chip
    return true;
  }

  Serial.println("[MPU] Init OK  ✓");
  return true;
}

// ═════════════════════════════════════════════════════════════
//  BMP280 INIT  (Martin Lindup BMP280_DEV)
// ═════════════════════════════════════════════════════════════
// BMP280 I2C address being used (set during init)
uint8_t bmpAddr = 0x76;

bool initBMP280() {
  // ── Step 1: detect address & read chip ID directly ──────
  uint8_t addr = 0xFF;
  uint8_t chipID = 0x00;

  for (uint8_t a : {(uint8_t)0x76, (uint8_t)0x77}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      // Read chip ID register 0xD0
      Wire.beginTransmission(a);
      Wire.write(0xD0);
      Wire.endTransmission(false);
      Wire.requestFrom(a, (uint8_t)1);
      if (Wire.available()) chipID = Wire.read();
      addr = a;
      Serial.printf("[BMP] Found at 0x%02X  chip_id=0x%02X\n", a, chipID);
      break;
    }
  }

  if (addr == 0xFF) {
    Serial.println("[BMP] No device on 0x76 or 0x77");
    return false;
  }

  // chip_id: 0x60 = BME280, 0x58 = BMP280, 0x56/0x57 = BMP280 sample
  if (chipID == 0x60) {
    Serial.println("[BMP] Chip is BME280 (humidity sensor) – BMP280_DEV still works");
  } else if (chipID == 0x58 || chipID == 0x56 || chipID == 0x57) {
    Serial.println("[BMP] Chip is genuine BMP280");
  } else {
    Serial.printf("[BMP] WARNING: unknown chip_id 0x%02X – clone chip\n", chipID);
  }

  bmpAddr = addr;

  // ── Step 2: soft-reset the chip (reg 0xE0 = 0xB6) ──────
  Wire.beginTransmission(addr);
  Wire.write(0xE0); Wire.write(0xB6);
  Wire.endTransmission();
  delay(100);

  // ── Step 3: init via library ─────────────────────────────
  bool libOK = false;
  if (addr == BMP280_I2C_ADDR)     libOK = bmp280.begin(BMP280_I2C_ADDR);
  else                             libOK = bmp280.begin(BMP280_I2C_ALT_ADDR);

  if (!libOK) {
    Serial.println("[BMP] Library begin() failed – attempting forced config");
    // Fall through; we will configure registers manually below
  }

  // ── Step 4: write config registers directly ──────────────
  // osrs_t=x2, osrs_p=x4, mode=normal  → reg 0xF4 = 0b01001111 = 0x4F
  // t_sb=62.5ms, filter=4, spi3w=0     → reg 0xF5 = 0b00010100 = 0x14
  Wire.beginTransmission(addr);
  Wire.write(0xF5); Wire.write(0x14);  // config: standby 62.5ms, filter coeff 4
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(addr);
  Wire.write(0xF4); Wire.write(0x4F);  // ctrl_meas: osrs_t=x2, osrs_p=x4, normal mode
  Wire.endTransmission();

  delay(600);  // wait for first measurement

  // ── Step 5: verify with raw register read ────────────────
  // Registers 0xF7-0xFC: press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb
  Wire.beginTransmission(addr);
  Wire.write(0xF7);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)6);
  if (Wire.available() >= 6) {
    uint8_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();
    int32_t rawP = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    int32_t rawT = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
    Serial.printf("[BMP] Raw register read: rawP=%d  rawT=%d\n", rawP, rawT);
    if (rawT == 0 || rawT == 0x80000) {
      Serial.println("[BMP] WARNING: raw temp is 0 – sensor not measuring. Check VCC=3.3V");
    }
  }

  // ── Step 6: try library getMeasurements ──────────────────
  if (libOK) {
    bmp280.setPresOversampling(OVERSAMPLING_X4);
    bmp280.setTempOversampling(OVERSAMPLING_X2);
    bmp280.setIIRFilter(IIR_FILTER_4);
    bmp280.setTimeStandby(TIME_STANDBY_250MS);
    bmp280.startNormalConversion();
    delay(300);

    float t, p, a;
    if (bmp280.getMeasurements(t, p, a)) {
      Serial.printf("[BMP] Library read OK – T=%.2f°C  P=%.2fhPa  Alt=%.2fm\n", t, p, a);
    } else {
      Serial.println("[BMP] Library getMeasurements() still returns false");
      Serial.println("[BMP] Will use raw register reads in loop instead");
      libOK = false;
    }
  }

  // store whether to use library or raw reads
  // reuse bmpAddr != 0xFF as "found", libOK stored in global
  return true;  // chip found either way
}

// ── flag: use raw I2C reads if library getMeasurements fails ─
bool bmpUseRaw = false;

// ── Raw BMP280 calibration data (needed for raw conversion) ──
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t  t_fine;

void loadBMPCalibration() {
  Wire.beginTransmission(bmpAddr);
  Wire.write(0x88);  // calib start register
  Wire.endTransmission(false);
  Wire.requestFrom(bmpAddr, (uint8_t)24);
  if (Wire.available() < 24) { Serial.println("[BMP] Calibration read failed"); return; }
  uint8_t c[24];
  for (int i = 0; i < 24; i++) c[i] = Wire.read();
  dig_T1 = (c[1]<<8)|c[0]; dig_T2=(c[3]<<8)|c[2]; dig_T3=(c[5]<<8)|c[4];
  dig_P1 = (c[7]<<8)|c[6]; dig_P2=(c[9]<<8)|c[8]; dig_P3=(c[11]<<8)|c[10];
  dig_P4 =(c[13]<<8)|c[12];dig_P5=(c[15]<<8)|c[14];dig_P6=(c[17]<<8)|c[16];
  dig_P7 =(c[19]<<8)|c[18];dig_P8=(c[21]<<8)|c[20];dig_P9=(c[23]<<8)|c[22];
  Serial.println("[BMP] Calibration loaded");
}

float bmpRawToTemp(int32_t rawT) {
  int32_t v1 = ((((rawT>>3)-((int32_t)dig_T1<<1)))*((int32_t)dig_T2))>>11;
  int32_t v2 = (((((rawT>>4)-((int32_t)dig_T1))*((rawT>>4)-((int32_t)dig_T1)))>>12)*((int32_t)dig_T3))>>14;
  t_fine = v1+v2;
  return (float)((t_fine*5+128)>>8) / 100.0f;
}

float bmpRawToPress(int32_t rawP) {
  int64_t v1 = ((int64_t)t_fine) - 128000;
  int64_t v2 = v1*v1*(int64_t)dig_P6;
  v2 = v2 + ((v1*(int64_t)dig_P5)<<17);
  v2 = v2 + (((int64_t)dig_P4)<<35);
  v1 = ((v1*v1*(int64_t)dig_P3)>>8)+((v1*(int64_t)dig_P2)<<12);
  v1 = (((((int64_t)1)<<47)+v1))*((int64_t)dig_P1)>>33;
  if (v1 == 0) return 0;
  int64_t p = 1048576 - rawP;
  p = (((p<<31)-v2)*3125)/v1;
  v1 = (((int64_t)dig_P9)*(p>>13)*(p>>13))>>25;
  v2 = (((int64_t)dig_P8)*p)>>19;
  p = ((p+v1+v2)>>8)+(((int64_t)dig_P7)<<4);
  return (float)p / 25600.0f;  // hPa
}

// ═════════════════════════════════════════════════════════════
//  CAMERA INIT
// ═════════════════════════════════════════════════════════════
bool initCamera() {
  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0  = Y2_GPIO_NUM; cfg.pin_d1 = Y3_GPIO_NUM;
  cfg.pin_d2  = Y4_GPIO_NUM; cfg.pin_d3 = Y5_GPIO_NUM;
  cfg.pin_d4  = Y6_GPIO_NUM; cfg.pin_d5 = Y7_GPIO_NUM;
  cfg.pin_d6  = Y8_GPIO_NUM; cfg.pin_d7 = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    cfg.frame_size = FRAMESIZE_VGA; cfg.jpeg_quality = 10; cfg.fb_count = 2;
  } else {
    cfg.frame_size = FRAMESIZE_QVGA; cfg.jpeg_quality = 12; cfg.fb_count = 1;
  }

  if (esp_camera_init(&cfg) != ESP_OK) {
    Serial.println("[CAM] Init FAILED"); return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, 1); s->set_brightness(s, 1); s->set_saturation(s, -1);
  Serial.println("[CAM] Init OK");
  return true;
}

// ═════════════════════════════════════════════════════════════
//  MQTT CONNECT / RECONNECT
// ═════════════════════════════════════════════════════════════
void mqttReconnect() {
  if (mqtt.connected()) return;
  Serial.print("[MQTT] Connecting...");

  char willMsg[80];
  snprintf(willMsg, sizeof(willMsg),
           "{\"status\":\"offline\",\"ip\":\"%s\"}",
           WiFi.localIP().toString().c_str());

  if (mqtt.connect(MQTT_CLIENT, NULL, NULL, TOPIC_STATUS, 1, true, willMsg)) {
    Serial.println(" connected!");
    char onlineMsg[100];
    snprintf(onlineMsg, sizeof(onlineMsg),
             "{\"status\":\"online\",\"ip\":\"%s\",\"uptime\":%lu}",
             WiFi.localIP().toString().c_str(), millis() / 1000UL);
    mqtt.publish(TOPIC_STATUS, onlineMsg, true);
  } else {
    Serial.printf(" failed rc=%d, retry in 5s\n", mqtt.state());
  }
}

void publishSensors() {
  if (!mqtt.connected()) return;
  char buf[300];

  // BMP280
  snprintf(buf, sizeof(buf),
    "{\"temperature\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,\"ok\":%s}",
    sData.temperature, sData.pressure, sData.altitude,
    sData.bmpOK ? "true" : "false"
  );
  mqtt.publish(TOPIC_BMP, buf);

  // MPU6050
  snprintf(buf, sizeof(buf),
    "{\"accelX\":%.4f,\"accelY\":%.4f,\"accelZ\":%.4f,"
    "\"gyroX\":%.3f,\"gyroY\":%.3f,\"gyroZ\":%.3f,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"ok\":%s}",
    sData.accelX, sData.accelY, sData.accelZ,
    sData.gyroX,  sData.gyroY,  sData.gyroZ,
    sData.roll,   sData.pitch,
    sData.mpuOK ? "true" : "false"
  );
  mqtt.publish(TOPIC_MPU, buf);

  // Camera / system status
  snprintf(buf, sizeof(buf),
    "{\"camOK\":%s,\"sdOK\":false,\"bmpOK\":%s,\"mpuOK\":%s,"
    "\"uptime\":%lu,\"rssi\":%d}",
    sData.camOK ? "true" : "false",
    sData.bmpOK ? "true" : "false",
    sData.mpuOK ? "true" : "false",
    millis() / 1000UL,
    WiFi.RSSI()
  );
  mqtt.publish(TOPIC_CAM, buf);
}

// ═════════════════════════════════════════════════════════════
//  SENSOR READ  (every 500 ms)
// ═════════════════════════════════════════════════════════════
void readSensors() {
  // BMP280 – try library first, fall back to raw registers
  if (sData.bmpOK) {
    bool gotReading = false;

    if (!bmpUseRaw) {
      float t, p, a;
      if (bmp280.getMeasurements(t, p, a)) {
        sData.temperature = t;
        sData.pressure    = p;
        sData.altitude    = a;
        gotReading = true;
      } else {
        // Library failed 3 times in a row → switch to raw mode permanently
        static uint8_t libFails = 0;
        libFails++;
        if (libFails >= 3) {
          Serial.println("[BMP] Switching to raw register reads permanently");
          loadBMPCalibration();
          bmpUseRaw = true;
        }
      }
    }

    if (bmpUseRaw && !gotReading) {
      // Burst read raw pressure + temp registers
      Wire.beginTransmission(bmpAddr);
      Wire.write(0xF7);
      Wire.endTransmission(false);
      Wire.requestFrom(bmpAddr, (uint8_t)6);
      if (Wire.available() >= 6) {
        uint8_t b[6];
        for (int i = 0; i < 6; i++) b[i] = Wire.read();
        int32_t rawP = ((int32_t)b[0]<<12)|((int32_t)b[1]<<4)|(b[2]>>4);
        int32_t rawT = ((int32_t)b[3]<<12)|((int32_t)b[4]<<4)|(b[5]>>4);
        sData.temperature = bmpRawToTemp(rawT);
        float pressPa     = bmpRawToPress(rawP);
        sData.pressure    = pressPa;
        // altitude from barometric formula (sea level = 1013.25 hPa)
        sData.altitude    = 44330.0f * (1.0f - powf(pressPa / 1013.25f, 0.1903f));
      }
    }
  }

  // MPU6050 – read raw registers directly (bypasses any library quirk)
  if (sData.mpuOK) {
    // Burst read 14 bytes starting at ACCEL_XOUT_H (0x3B)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

    if (Wire.available() >= 14) {
      int16_t ax = (Wire.read() << 8) | Wire.read();
      int16_t ay = (Wire.read() << 8) | Wire.read();
      int16_t az = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();   // skip temperature
      int16_t gx = (Wire.read() << 8) | Wire.read();
      int16_t gy = (Wire.read() << 8) | Wire.read();
      int16_t gz = (Wire.read() << 8) | Wire.read();

      sData.accelX = ax / 16384.0f;   // ±2g
      sData.accelY = ay / 16384.0f;
      sData.accelZ = az / 16384.0f;
      sData.gyroX  = gx / 131.0f;    // ±250°/s
      sData.gyroY  = gy / 131.0f;
      sData.gyroZ  = gz / 131.0f;

      sData.roll  = atan2f(sData.accelY, sData.accelZ) * 180.0f / PI;
      sData.pitch = atan2f(-sData.accelX,
                    sqrtf(sData.accelY * sData.accelY +
                          sData.accelZ * sData.accelZ)) * 180.0f / PI;
    }
  }
}

// ═════════════════════════════════════════════════════════════
//  HTTP HANDLERS
// ═════════════════════════════════════════════════════════════
void handleSensors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache");
  char json[620];
  snprintf(json, sizeof(json),
    "{\"temperature\":%.2f,\"pressure\":%.2f,\"altitude\":%.2f,"
     "\"accelX\":%.4f,\"accelY\":%.4f,\"accelZ\":%.4f,"
     "\"gyroX\":%.3f,\"gyroY\":%.3f,\"gyroZ\":%.3f,"
     "\"roll\":%.2f,\"pitch\":%.2f,"
     "\"bmpOK\":%s,\"mpuOK\":%s,\"sdOK\":false,\"camOK\":%s,"
     "\"uptime\":%lu}",
    sData.temperature, sData.pressure, sData.altitude,
    sData.accelX, sData.accelY, sData.accelZ,
    sData.gyroX,  sData.gyroY,  sData.gyroZ,
    sData.roll, sData.pitch,
    sData.bmpOK ? "true":"false",
    sData.mpuOK ? "true":"false",
    sData.camOK ? "true":"false",
    millis() / 1000UL
  );
  server.send(200, "application/json", json);
}

// ── Stream task runs on its own FreeRTOS task (Core 0) ──────
// This prevents the MJPEG stream from blocking sensor HTTP requests
struct StreamTaskArgs { WiFiClient client; };

void streamTask(void* param) {
  WiFiClient client = ((StreamTaskArgs*)param)->client;
  delete (StreamTaskArgs*)param;

  client.print(
    "HTTP/1.1 200 OK\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
    "Connection: keep-alive\r\n\r\n"
  );

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(50 / portTICK_PERIOD_MS); continue; }

    client.printf(
      "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      (unsigned)fb->len
    );
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    vTaskDelay(80 / portTICK_PERIOD_MS);  // ~12 fps, gives other tasks CPU time
  }
  client.stop();
  vTaskDelete(NULL);
}

void handleStream() {
  StreamTaskArgs* args = new StreamTaskArgs();
  args->client = server.client();
  // Pin stream to Core 0, sensor loop runs on Core 1 (Arduino default)
  xTaskCreatePinnedToCore(streamTask, "mjpeg_stream", 8192, args, 1, NULL, 0);
}

void handleRoot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String ip = WiFi.localIP().toString();
  server.send(200, "text/html",
    "<!DOCTYPE html><html><body>"
    "<h2>ESP32-CAM Sensor Node v3</h2>"
    "<p>IP: <b>" + ip + "</b></p>"
    "<ul><li><a href='/stream'>Stream</a></li>"
    "<li><a href='/sensors'>Sensors JSON</a></li></ul>"
    "</body></html>");
}

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n========================================");
  Serial.println("  ESP32-CAM Sensor Node  v3");
  Serial.println("========================================");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // I2C – 100 kHz is reliable for breadboard jumper wires
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(200);

  i2cScan();   // print all found I2C addresses first

  // Sensors before camera (camera init can take a moment)
  sData.bmpOK = initBMP280();
  sData.mpuOK = initMPU6050();
  sData.camOK = initCamera();

  sData.sdOK = false;
  Serial.println("[SD]  No SD card → UNAVAILABLE");

  // WiFi
  Serial.printf("\n[WiFi] Connecting to \"%s\"", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t att = 0;
  while (WiFi.status() != WL_CONNECTED && att < 40) {
    delay(500); Serial.print("."); att++;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
  digitalWrite(LED_PIN, LOW);

  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\n[WiFi] IP = %s  RSSI = %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  else
    Serial.println("\n[WiFi] Connection FAILED");

  server.on("/",        HTTP_GET, handleRoot);
  server.on("/stream",  HTTP_GET, handleStream);
  server.on("/sensors", HTTP_GET, handleSensors);
  server.on("/status",  HTTP_GET, handleSensors);
  server.begin();

  // MQTT setup
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setKeepAlive(15);
  mqtt.setBufferSize(512);
  mqttReconnect();
  Serial.printf("[MQTT] Broker: %s:%d\n", MQTT_BROKER, MQTT_PORT);

  Serial.println("========================================");
  Serial.printf("  Stream  : http://%s/stream\n",  WiFi.localIP().toString().c_str());
  Serial.printf("  Sensors : http://%s/sensors\n", WiFi.localIP().toString().c_str());
  Serial.println("========================================\n");
}

// ═════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════
unsigned long lastRead    = 0;
unsigned long lastMqtt    = 0;
unsigned long lastReconn  = 0;

void loop() {
  server.handleClient();

  // MQTT keep-alive
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected() && millis() - lastReconn > 5000) {
      mqttReconnect();
      lastReconn = millis();
    }
    mqtt.loop();
  }

  // Read sensors every 500 ms
  if (millis() - lastRead >= 500) {
    readSensors();
    lastRead = millis();
  }

  // Publish MQTT every 1000 ms
  if (millis() - lastMqtt >= 1000) {
    publishSensors();
    lastMqtt = millis();
  }
}
