# IoT Real-Time UV Monitoring System (0–3.3V Full-Scale Calibration)

An end-to-end Internet of Things (IoT) ultraviolet (UV) radiation monitoring platform built on the **ESP32-C3** microcontroller, a **GUVA-S12SD** UV photodetector module, an **SSD1306 OLED** display, cloud telemetry (**HiveMQ MQTT** & **InfluxDB Cloud**), and a cross-platform **Flutter** mobile application.

---

## 📁 Repository Structure

```text
Final-3.3/
├── .github/
│   └── workflows/
│       └── build_apk.yml        # Automated GitHub Actions CI/CD pipeline to compile the Android APK
├── mobile_app/
│   ├── lib/
│   │   └── main.dart            # Synchronized Flutter application source code
│   └── pubspec.yaml             # Flutter dependencies (fl_chart, mqtt_client, http, share_plus, etc.)
├── src/
│   └── main.cpp                 # ESP32-C3 PlatformIO C++ firmware (ADC, DSP filters, OLED, HTTP, MQTT, InfluxDB)
├── app1.dart                    # Master Flutter dashboard source file (synced to mobile_app/lib/main.dart)
├── platformio.ini               # PlatformIO build configuration & library dependencies for ESP32-C3
├── .gitignore                   # Excludes build artifacts (.pio, build, .dart_tool, local.properties)
└── README.md                    # Project architecture, hardware setup, and calibration documentation
```

---

## ⚙️ System Architecture & Data Flow

1. **Edge Acquisition (`src/main.cpp`):**
   * Samples the analog UV sensor voltage on `GPIO 2` (`ADC1_CH2`) at 12-bit resolution (`0–4095`) using `32x` multisampling every `500 ms`.
   * Converts the raw ADC count into millivolts (`0–3300 mV`) using the $3.3\text{ V} / 4095.0$ ratio.
2. **3-Tier Digital Signal Processing (DSP):**
   * **Tier 1 — 9-Point Median Filter:** Eliminates sudden Wi-Fi RF transmission spikes and transient electrical noise.
   * **Tier 2 — 1D Kalman Filter:** Smooths continuous white Gaussian ADC noise.
   * **Tier 3 — Hybrid Filter (Median + Kalman):** Feeds the spike-free median output into a secondary Kalman estimator for stable, low-latency UV readings, with an instant zero-reset below the `50.0 mV` indoor noise floor.
3. **Local & Cloud Telemetry:**
   * **OLED Display (`SSD1306` via I2C):** Displays the local IP address, Hybrid filtered millivolts (`Hyb mV`), calibrated UV Index (`UV Idx`), WHO risk category, and Wi-Fi RSSI in real time.
   * **Local HTTP REST Server (`/data`):** Serves real-time JSON telemetry on port `80` for low-latency local polling.
   * **HiveMQ Public MQTT Broker (`broker.hivemq.com`):** Publishes JSON payloads to `venkat_iot/sensors/uv/node_01` for instant remote mobile updates.
   * **InfluxDB Cloud v2:** Streams time-series measurements (`voltage_raw_mv`, `voltage_hybrid_mv`, `uv_index_estimated`) to the `uv_sensor_data` bucket for long-term historical persistence.

---

## 🔌 Hardware Wiring (ESP32-C3 SuperMini / DevKitM-1)

| Component | Pin | ESP32-C3 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **GUVA-S12SD UV Sensor** | `VCC` | `3V3` | 3.3V Regulated Power Rail |
| **GUVA-S12SD UV Sensor** | `GND` | `GND` | Common Ground |
| **GUVA-S12SD UV Sensor** | `SIG / OUT` | `GPIO 2` | Analog Input (`ADC1_CH2`, `ADC_11db`) |
| **SSD1306 OLED (128x64)** | `VDD` | `3V3` | 3.3V Power |
| **SSD1306 OLED (128x64)** | `GND` | `GND` | Common Ground |
| **SSD1306 OLED (128x64)** | `SDA` | `GPIO 8` | I2C Data Line |
| **SSD1306 OLED (128x64)** | `SCK / SCL` | `GPIO 9` | I2C Clock Line |

---

## 📐 0–3.3V (0–3300 mV) Linear UV Index Calibration

Both the ESP32-C3 firmware (`src/main.cpp`) and the Flutter mobile application (`app1.dart`) map the full **0–3300 mV (`0–3.3 V`)** sensor output linearly across the **0.0 to 12.0 UV Index** scale ($275\text{ mV}$ per $1.0\text{ UVI}$):

$$\text{UV Index} = \left(\frac{\text{Voltage}_{\text{mV}}}{3300.0}\right) \times 12.0$$

*Readings below `50.0 mV` are clamped to `0.0 mV` (`UVI 0.0`) to eliminate indoor dark-current noise.*

| UV Index | Sensor Output (`mV`) | Voltage (`V`) | WHO Risk Category | Recommended Protection |
| :--- | :--- | :--- | :--- | :--- |
| **UVI 0** | `< 50 mV` | `< 0.05 V` | **Low** | No protection required |
| **UVI 1** | `275 mV` | `0.275 V` | **Low** | Safe for outdoor activity |
| **UVI 2** | `550 mV` | `0.550 V` | **Low** | Safe for outdoor activity |
| **UVI 3** | `825 mV` | `0.825 V` | **Moderate** | Apply SPF 30+ sunscreen |
| **UVI 4** | `1100 mV` | `1.100 V` | **Moderate** | Seek shade near midday |
| **UVI 5** | `1375 mV` | `1.375 V` | **Moderate** | Hat, sunglasses & SPF 30+ |
| **UVI 6** | `1650 mV` | `1.650 V` | **High** | Reduce sun exposure 10 AM–4 PM |
| **UVI 7** | `1925 mV` | `1.925 V` | **High** | Protective clothing & SPF 30+ |
| **UVI 8** | `2200 mV` | `2.200 V` | **Very High** | Extra precautions required |
| **UVI 9** | `2475 mV` | `2.475 V` | **Very High** | Minimize midday sun exposure |
| **UVI 10** | `2750 mV` | `2.750 V` | **Very High** | Unprotected skin burns quickly |
| **UVI 11** | `3025 mV` | `3.025 V` | **Extreme** | Avoid direct sun exposure |
| **UVI 12** | `3300 mV` | `3.300 V` | **Extreme** | Maximum full-scale UV radiation |

---

## 📱 Flutter Mobile Application Features (`app1.dart`)

* **Multi-Source Sync Engine:** Automatically merges real-time packets from Local HTTP (`/data`), HiveMQ MQTT (`broker.hivemq.com`), and historical 24-hour queries from InfluxDB Cloud.
* **Google Weather-Inspired UI:** Features a 7-day calendar selector, semi-circular WHO gradient arc gauge, Dark/Light mode toggle, and personal UV sun-protection guidance.
* **Interactive Scrollable WHO UV Curve:** Horizontally scrollable gradient area chart with selectable time buckets (`Seconds`, `Minutes`, `Hours`, `Days`, `Months`) and optional raw-signal overlay.
* **Hourly UV Forecast & Peak Ranking:** Displays a 24-hour horizontal capsule strip (`00:00`–`23:00`) alongside a descending peak-UV ranking list.
* **DSP Filter Benchmarking & Data Export:** Compares `Hybrid (Median+Kalman)`, `1D Kalman Filter`, and `Median Filter` standard deviations ($\sigma$) in real time, and exports unclipped high-resolution PNG graphs or CSV datasets.

---

## 🚀 Build & Deployment Instructions

### 1. Flashing the ESP32-C3 Firmware (PlatformIO)
1. Open the repository root folder in **VS Code** with the **PlatformIO IDE** extension installed.
2. Update `WIFI_SSID` and `WIFI_PASSWORD` in `src/main.cpp` if needed.
3. Connect the ESP32-C3 via USB and run:
   ```bash
   pio run --target upload
   ```

### 2. Building the Android APK (GitHub Actions CI/CD)
Every push to the `main` branch automatically triggers `.github/workflows/build_apk.yml`:
1. Navigate to the **Actions** tab in your GitHub repository.
2. Select the latest **Build Final 3.3V UV Monitor APK** workflow run.
3. Download the **`Final-3.3V-UV-Monitor-APK`** artifact at the bottom of the summary page and install `app-release.apk` on your Android device.