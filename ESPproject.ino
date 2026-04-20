#include <Wire.h>
#include <RTClib.h>
#include "DHT.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <LPS.h>
#include <SD.h>
#include <SPI.h>

#define DHTPIN  13
#define DHTTYPE DHT11
#define SD_CS   2
#define INTERVAL_RTC  1000   // DS1307  : 1 Hz
#define INTERVAL_DHT  1000   // DHT11   : 1 Hz (hardware max)
#define INTERVAL_MPU   100   // MPU6050 : 10 Hz
#define INTERVAL_LSM    50   // LSM6DSO : 20 Hz
#define INTERVAL_LIS   100   // LIS3MDL : 10 Hz
#define INTERVAL_LPS    50   // LPS22DF : 20 Hz

RTC_DS1307     rtc;
DHT            dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
LSM6           lsm;
LIS3MDL        lis;
LPS            lps;

bool rtc_ok = false, mpu_ok = false, lsm_ok = false, lis_ok = false, lps_ok = false, sd_ok = false;
unsigned long lastRTC = 0, lastDHT = 0, lastMPU = 0, lastLSM = 0, lastLIS = 0, lastLPS = 0;
float lastTemp = 0, lastHumidity = 0;

const char* const days[] PROGMEM = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
const char* dayName(uint8_t d) {
  return (d < 7) ? days[d] : "?";
}

void printDiv() {
  Serial.println(F("============================================================"));
}
void printSection(const char* t) {
  Serial.println(F("------------------------------------------------------------"));
  Serial.print(F("  [ ")); Serial.print(t); Serial.println(F(" ]"));
  Serial.println(F("------------------------------------------------------------"));
}

void printTwoDigit(int val) {
  if (val < 10) Serial.print('0');
  Serial.print(val);
}

struct DHTData  { float temp, humidity;                                     bool ready = false; };
struct MPUData  { float ax, ay, az, gx, gy, gz, temp;                       bool ready = false; };
struct LSMData  { float ax, ay, az, gx, gy, gz;                             bool ready = false; };
struct LISData  { float mx, my, mz;                                         bool ready = false; };
struct LPSData  { float pressure, altitude, temp;                           bool ready = false; };

DHTData dhtBuf;
MPUData mpuBuf;
LSMData lsmBuf;
LISData lisBuf;
LPSData lpsBuf;

void flushSD() {
  if (!sd_ok) { Serial.println(F("  [flush] SD not ok")); return; }
  if (!dhtBuf.ready) { Serial.println(F("  [flush] waiting: DHT")); return; }
  if (!mpuBuf.ready) { Serial.println(F("  [flush] waiting: MPU")); return; }
  if (!lsmBuf.ready) { Serial.println(F("  [flush] waiting: LSM")); return; }
  if (!lisBuf.ready) { Serial.println(F("  [flush] waiting: LIS")); return; }
  if (!lpsBuf.ready) { Serial.println(F("  [flush] waiting: LPS")); return; }
  File f = SD.open("/log.csv", FILE_APPEND);
  if (!f) return;
  char ts[20];
  if (rtc_ok) {
    DateTime t = rtc.now();
    snprintf(ts, sizeof(ts), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             t.year(), t.month(), t.day(), t.hour(), t.minute(), t.second());
  } else {
    snprintf(ts, sizeof(ts), "%lu", millis()); // fallback
  }
  f.print(ts);
  f.print(','); f.print(dhtBuf.temp, 2);      f.print(','); f.print(dhtBuf.humidity, 2);
  f.print(','); f.print(mpuBuf.ax, 3);        f.print(','); f.print(mpuBuf.ay, 3);       f.print(','); f.print(mpuBuf.az, 3);
  f.print(','); f.print(mpuBuf.gx, 3);        f.print(','); f.print(mpuBuf.gy, 3);       f.print(','); f.print(mpuBuf.gz, 3);
  f.print(','); f.print(mpuBuf.temp, 2);
  f.print(','); f.print(lsmBuf.ax, 3);        f.print(','); f.print(lsmBuf.ay, 3);       f.print(','); f.print(lsmBuf.az, 3);
  f.print(','); f.print(lsmBuf.gx, 2);        f.print(','); f.print(lsmBuf.gy, 2);       f.print(','); f.print(lsmBuf.gz, 2);
  f.print(','); f.print(lisBuf.mx, 4);        f.print(','); f.print(lisBuf.my, 4);       f.print(','); f.print(lisBuf.mz, 4);
  f.print(','); f.print(lpsBuf.pressure, 2);  f.print(','); f.print(lpsBuf.altitude, 1); f.print(','); f.println(lpsBuf.temp, 2);
  f.close();
  dhtBuf.ready = mpuBuf.ready = lsmBuf.ready = lisBuf.ready = lpsBuf.ready = false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000); // Non-blocking wait
  Wire.begin(21, 22);
  dht.begin();

  if (SD.begin(SD_CS)) {
    sd_ok = true;
    File f = SD.open("/log.csv", FILE_WRITE);
    if (f) { f.println(F("utc,dht_temp_C,dht_humidity_pct,mpu_ax,mpu_ay,mpu_az,mpu_gx,mpu_gy,mpu_gz,mpu_temp_C,lsm_ax,lsm_ay,lsm_az,lsm_gx,lsm_gy,lsm_gz,lis_mx,lis_my,lis_mz,lps_pressure_hPa,lps_altitude_m,lps_temp_C")); f.close(); }
  }

  printDiv();
  Serial.println(F("  SUSF CanSat Sensor Suite - ESP32"));
  printDiv();
  Serial.println(sd_ok ? F("  SD Card : OK") : F("  SD Card : NOT FOUND"));

  rtc_ok = rtc.begin();
  Serial.println(rtc_ok ? F("  DS1307 : OK") : F("  DS1307 : NOT FOUND"));
  
  Serial.println(F("  DHT11  : OK"));

  if (mpu.begin(0x69, &Wire)) {
    mpu_ok = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("  MPU6050: OK"));
  } else {
    Serial.println(F("  MPU6050: NOT FOUND"));
  }

  if (lsm.init()) {
    lsm.enableDefault();
    lsm_ok = true;
    Serial.println(F("  LSM6DSO: OK"));
  } else {
    Serial.println(F("  LSM6DSO: NOT FOUND"));
  }

  if (lis.init()) {
    lis.enableDefault();
    lis_ok = true;
    Serial.println(F("  LIS3MDL: OK"));
  } else {
    Serial.println(F("  LIS3MDL: NOT FOUND"));
  }

  if (lps.init()) {
    lps.enableDefault();
    lps_ok = true;
    Serial.println(F("  LPS22DF: OK"));
  } else {
    Serial.println(F("  LPS22DF: NOT FOUND"));
  }

  printDiv();
  unsigned long t = millis();
  lastRTC = lastDHT = lastMPU = lastLSM = lastLIS = lastLPS = t;
}

uint32_t loopCount = 0;

void loop() {
  unsigned long now = millis();

  if (now - lastRTC >= INTERVAL_RTC) {
    lastRTC = now;
    printSection("DS1307  Real-Time Clock");
    if (rtc_ok) {
      DateTime t = rtc.now();
      Serial.print(F("  Date : "));
      Serial.print(dayName(t.dayOfTheWeek())); Serial.print(' ');
      printTwoDigit(t.day()); Serial.print('/');
      printTwoDigit(t.month()); Serial.print('/');
      Serial.println(t.year());
      Serial.print(F("  Time : "));
      printTwoDigit(t.hour()); Serial.print(':');
      printTwoDigit(t.minute()); Serial.print(':');
      printTwoDigit(t.second()); Serial.println();
    } else {
      Serial.println(F("  !! DS1307 not available"));
    }
  }

  if (now - lastDHT >= INTERVAL_DHT) {
    lastDHT = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h) && !isnan(t)) { lastTemp = t; lastHumidity = h; }
    printSection("DHT11  Temperature & Humidity");
    if (lastTemp != 0 || lastHumidity != 0) {
      Serial.print(F("  Temperature : ")); Serial.print(lastTemp, 1); Serial.println(F(" C"));
      Serial.print(F("  Humidity    : ")); Serial.print(lastHumidity, 1); Serial.println(F(" %RH"));
      dhtBuf = { lastTemp, lastHumidity, true };
      flushSD();
    } else {
      Serial.println(F("  !! DHT11 read failed"));
    }
  }

  if (now - lastMPU >= INTERVAL_MPU) {
    lastMPU = now;
    printSection("MPU-6050  Accelerometer & Gyroscope");
    if (mpu_ok) {
      sensors_event_t accel, gyro, temp;
      mpu.getEvent(&accel, &gyro, &temp);
      Serial.print(F("  Accel  X:")); Serial.print(accel.acceleration.x, 2);
      Serial.print(F("  Y:")); Serial.print(accel.acceleration.y, 2);
      Serial.print(F("  Z:")); Serial.print(accel.acceleration.z, 2); Serial.println(F(" m/s2"));
      Serial.print(F("  Gyro   X:")); Serial.print(gyro.gyro.x, 3);
      Serial.print(F("  Y:")); Serial.print(gyro.gyro.y, 3);
      Serial.print(F("  Z:")); Serial.print(gyro.gyro.z, 3); Serial.println(F(" rad/s"));
      Serial.print(F("  Temp   : ")); Serial.print(temp.temperature, 1); Serial.println(F(" C"));
      mpuBuf = { accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                 gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, temp.temperature, true };
      flushSD();
    } else {
      Serial.println(F("  !! MPU-6050 not available"));
    }
  }

  if (now - lastLSM >= INTERVAL_LSM) {
    lastLSM = now;
    printSection("LSM6DSO  Accelerometer & Gyroscope");
    if (lsm_ok) {
      lsm.read();
      Serial.print(F("  Accel  X:")); Serial.print(lsm.a.x * 0.000122f, 3);
      Serial.print(F("  Y:")); Serial.print(lsm.a.y * 0.000122f, 3);
      Serial.print(F("  Z:")); Serial.print(lsm.a.z * 0.000122f, 3); Serial.println(F(" g"));
      Serial.print(F("  Gyro   X:")); Serial.print(lsm.g.x * 0.01750f, 2);
      Serial.print(F("  Y:")); Serial.print(lsm.g.y * 0.01750f, 2);
      Serial.print(F("  Z:")); Serial.print(lsm.g.z * 0.01750f, 2); Serial.println(F(" deg/s"));
      lsmBuf = { lsm.a.x * 0.000122f, lsm.a.y * 0.000122f, lsm.a.z * 0.000122f,
                 lsm.g.x * 0.01750f,  lsm.g.y * 0.01750f,  lsm.g.z * 0.01750f, true };
      flushSD();
    } else {
      Serial.println(F("  !! LSM6DSO not available"));
    }
  }

  if (now - lastLIS >= INTERVAL_LIS) {
    lastLIS = now;
    printSection("LIS3MDL  Magnetometer");
    if (lis_ok) {
      lis.read();
      Serial.print(F("  Mag    X:")); Serial.print(lis.m.x / 6842.0f, 4);
      Serial.print(F("  Y:")); Serial.print(lis.m.y / 6842.0f, 4);
      Serial.print(F("  Z:")); Serial.print(lis.m.z / 6842.0f, 4); Serial.println(F(" G"));
      lisBuf = { lis.m.x / 6842.0f, lis.m.y / 6842.0f, lis.m.z / 6842.0f, true };
      flushSD();
    } else {
      Serial.println(F("  !! LIS3MDL not available"));
    }
  }

  if (now - lastLPS >= INTERVAL_LPS) {
    lastLPS = now;
    printSection("LPS22DF  Barometric Altimeter");
    if (lps_ok) {
      float pressure = lps.readPressureMillibars();
      float altitude = lps.pressureToAltitudeMeters(pressure);
      float tempLPS  = lps.readTemperatureC();
      Serial.print(F("  Pressure : ")); Serial.print(pressure, 2); Serial.println(F(" hPa"));
      Serial.print(F("  Altitude : ")); Serial.print(altitude, 1); Serial.println(F(" m"));
      Serial.print(F("  Temp     : ")); Serial.print(tempLPS, 2);  Serial.println(F(" C"));
      lpsBuf = { pressure, altitude, tempLPS, true };
      flushSD();
    } else {
      Serial.println(F("  !! LPS22DF not available"));
    }
  }
}