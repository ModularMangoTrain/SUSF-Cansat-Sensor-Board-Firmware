# SUSF CanSat — ESP32 Sensor Suite & DAQ

An embedded data acquisition (DAQ) system for a CanSat payload, running on an **ESP32**. It reads from six sensors at independent sampling rates, logs all data to an SD card as CSV, and streams a formatted report over Serial.

---

## Hardware

| Component | Interface | Role |
|-----------|-----------|------|
| ESP32 | — | Main MCU |
| DS1307 | I²C | Real-time clock (UTC timestamp) |
| DHT11 | GPIO 13 | Temperature & relative humidity |
| MPU-6050 | I²C (0x69) | 3-axis accelerometer + gyroscope |
| LSM6DSO | I²C | 3-axis accelerometer + gyroscope (high-rate) |
| LIS3MDL | I²C | 3-axis magnetometer |
| LPS22DF | I²C | Barometric pressure & altitude |
| MicroSD | SPI (CS: GPIO 2) | CSV data logging |

I²C bus: SDA → GPIO 21, SCL → GPIO 22

---

## Sampling Rates

| Sensor | Rate |
|--------|------|
| DS1307 / DHT11 | 1 Hz |
| MPU-6050 / LIS3MDL | 10 Hz |
| LSM6DSO / LPS22DF | 20 Hz |

---

## Data Logging

On startup, `log.csv` is created on the SD card with the following columns:

```
utc, dht_temp_C, dht_humidity_pct,
mpu_ax, mpu_ay, mpu_az, mpu_gx, mpu_gy, mpu_gz, mpu_temp_C,
lsm_ax, lsm_ay, lsm_az, lsm_gx, lsm_gy, lsm_gz,
lis_mx, lis_my, lis_mz,
lps_pressure_hPa, lps_altitude_m, lps_temp_C
```

A row is written once all six sensor buffers have a fresh reading (synchronized flush). If the RTC is unavailable, `millis()` is used as a fallback timestamp.

---

## Dependencies (Arduino Libraries)

Install via Arduino Library Manager or PlatformIO:

- `RTClib` — Adafruit
- `DHT sensor library` — Adafruit
- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `LSM6` — Pololu
- `LIS3MDL` — Pololu
- `LPS` — Pololu
- `SD` (built-in)
- `Wire` / `SPI` (built-in)

---

## Project Structure

```
ESPproject/
└── ESPproject.ino   # Main firmware
```

---

## Build & Flash

1. Open `ESPproject.ino` in the Arduino IDE (or PlatformIO).
2. Select board: **ESP32 Dev Module**.
3. Set upload speed to `115200`.
4. Flash and open Serial Monitor at **115200 baud**.
