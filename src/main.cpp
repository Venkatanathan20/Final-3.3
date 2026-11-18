// =========================================================================
// 1. LIBRARY IMPORTS & HARDWARE INTERFACES
// Purpose: Loads core Arduino framework, Wi-Fi connectivity, local HTTP web 
//          server, MQTT cloud telemetry, HTTPS client for InfluxDB, JSON 
//          serialization, and I2C drivers for the SSD1306 OLED screen.
// =========================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =========================================================================
// 2. LOCAL WI-FI NETWORK CREDENTIALS
// Purpose: Configures the SSID and password for the NTUST lab router (IB707)
//          so the ESP32-C3 can connect to the internet and local LAN.
// =========================================================================
const char* WIFI_SSID     = "IB707";
const char* WIFI_PASSWORD = "IB707IB707";

// =========================================================================
// 3. HARDWARE PINOUT & OLED DISPLAY DIMENSIONS
// Purpose: Maps the GUVA-S12SD analog UV sensor to ADC1 (GPIO 2, unaffected 
//          by Wi-Fi radio activity) and assigns I2C pins for the 128x64 OLED.
// =========================================================================
#define UV_SENSOR_PIN 2   // GUVA-S12SD Analog Signal Output (ADC1_CH2)
#define I2C_SDA_PIN   8   // OLED I2C Data Line (SDA)
#define I2C_SCL_PIN   9   // OLED I2C Clock Line (SCL)
#define SCREEN_WIDTH  128 // OLED Horizontal Resolution in Pixels
#define SCREEN_HEIGHT 64  // OLED Vertical Resolution in Pixels

// =========================================================================
// 4. PERIPHERAL & NETWORK CLIENT OBJECTS
// Purpose: Instantiates the OLED display driver, local REST API web server 
//          on port 80, and MQTT client wrapper over TCP Wi-Fi.
// =========================================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// =========================================================================
// 5. CLOUD TELEMETRY ENDPOINTS (HIVEMQ MQTT & INFLUXDB CLOUD)
// Purpose: Defines broker address, topic namespace, unique node identifier, 
//          and authenticated HTTPS write URL for time-series database logging.
// =========================================================================
const char* MQTT_BROKER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;
const char* MQTT_TOPIC    = "venkat_iot/sensors/uv/node_01";
const char* DEVICE_ID     = "ESP32_node_01";

#define INFLUXDB_URL    "https://us-east-1-1.aws.cloud2.influxdata.com/api/v2/write?org=IOT&bucket=uv_sensor_data&precision=s"
#define INFLUXDB_TOKEN  "Dbzt7IHP3G4E4McgOmsRBrZLUaK2fsEgexNVFzYSQI1V2PhqWbL6SFhB64Vg-jOY6HQIu7BibH8jkSQvHqLNWw=="

// =========================================================================
// 6. 3-TIER DIGITAL SIGNAL PROCESSING (DSP) & TELEMETRY STATE VARIABLES
// Purpose: Maintains sliding window buffers for the 9-point Median filter, 
//          state covariance for 1D Kalman & Hybrid filters, and live readings.
// =========================================================================
#define MEDIAN_WIN_SIZE 9          // Sliding window length to reject RF spikes
float winBuf[MEDIAN_WIN_SIZE];     // Circular buffer storing recent mV samples
int winCount = 0;                  // Number of valid samples currently in buffer
int winIdx   = 0;                  // Current write index in circular buffer

bool filterInitialized = false;    // Tracks first valid non-zero sample
float kalX = 0.0f, kalP = 1.0f;    // 1D Kalman estimate (X) and error covariance (P)
float hybX = 0.0f, hybP = 1.0f;    // Hybrid (Median+Kalman) estimate and covariance

unsigned long lastSampleMillis = 0;           // Timestamp of last sensor acquisition
const unsigned long SAMPLE_INTERVAL_MS = 500; // Sampling rate: 2 Hz (every 500 ms)
uint32_t packetSeq = 0;                       // Monotonic packet sequence counter

float currentMvRaw    = 0.0f;      // Unfiltered oversampled sensor voltage (mV)
float currentMvMedian = 0.0f;      // Stage 1: Median-filtered voltage (mV)
float currentMvKalman = 0.0f;      // Stage 2: Kalman-filtered voltage (mV)
float currentMvHybrid = 0.0f;      // Stage 3: Hybrid-filtered voltage (mV)
float currentUvIndex  = 0.0f;      // Calibrated UV Index (0.00 - 12.00 UVI)
String currentRisk    = "Low";     // Full WHO UV risk label for JSON/App
int currentRssi       = 0;         // Wi-Fi signal strength in dBm

// =========================================================================
// 7. FULL-SCALE 0–3.3V (0–3300 mV) UV INDEX & WHO RISK CLASSIFICATION
// Purpose: Maps the full 0–3300 mV analog range linearly to 0–12 UVI (275 mV
//          per UVI) with a 50 mV indoor dark-noise cutoff, and assigns WHO 
//          exposure categories for both mobile app and compact OLED screens.
// =========================================================================
float calculateFullScaleUVI(float mv) {
  if (mv < 50.0f) return 0.0f;               // Suppress indoor dark-current jitter
  float uvi = (mv / 3300.0f) * 12.0f;        // Linear 0-3.3V to 0-12 UVI scaling
  return (uvi > 12.0f) ? 12.0f : uvi;        // Cap upper bound at 12.0 UVI
}

String getRiskCategory(float uvi) {
  if (uvi < 3.0f)  return "Low";
  if (uvi < 6.0f)  return "Moderate";
  if (uvi < 8.0f)  return "High";
  if (uvi < 11.0f) return "Very high";
  return "Extreme";
}

String getShortOledRisk(float uvi) {
  if (uvi < 3.0f)  return "Low";
  if (uvi < 6.0f)  return "Mod";
  if (uvi < 8.0f)  return "High";
  if (uvi < 11.0f) return "V.High";          // Abbreviated to fit 1 OLED line cleanly
  return "Extreme";
}

// =========================================================================
// 8. HARDWARE ADC OVERSAMPLING & 3-TIER DSP FILTERING PIPELINE
// Purpose: Averages 32 consecutive 12-bit ADC samples (0–4095) into millivolts,
//          removes transient Wi-Fi RF spikes via Median filtering, and applies
//          Kalman smoothing for stable, jitter-free UV Index calculation.
// =========================================================================
float readAveragedMilliVolts() {
  uint32_t totalRaw = 0;
  for (int i = 0; i < 32; i++) {
    totalRaw += analogRead(UV_SENSOR_PIN);   // Read 12-bit raw ADC count (0..4095)
    delayMicroseconds(250);                  // Short settling delay between samples
  }
  float avgRaw = totalRaw / 32.0f;
  return (avgRaw * 3300.0f) / 4095.0f;       // Convert ADC count to 0–3300 mV scale
}

void updateFiltersAndSensors() {
  float raw = readAveragedMilliVolts();

  // Zero-clamp readings below the 50 mV indoor dark-noise threshold
  if (raw < 50.0f) {
    raw = 0.0f;
  }

  float clampedRaw = constrain(raw, 0.0f, 3300.0f);
  currentMvRaw = clampedRaw;

  // Stage 1: 9-Point Circular Buffer & Bubble Sort for Median Extraction
  winBuf[winIdx] = clampedRaw;
  winIdx = (winIdx + 1) % MEDIAN_WIN_SIZE;
  if (winCount < MEDIAN_WIN_SIZE) winCount++;

  float sorted[MEDIAN_WIN_SIZE];
  for (int i = 0; i < winCount; i++) sorted[i] = winBuf[i];
  for (int i = 0; i < winCount - 1; i++) {
    for (int j = i + 1; j < winCount; j++) {
      if (sorted[j] < sorted[i]) {
        float tmp = sorted[i]; sorted[i] = sorted[j]; sorted[j] = tmp;
      }
    }
  }
  currentMvMedian = sorted[winCount / 2];

  // Stage 2 & 3: 1D Kalman & Hybrid (Median + Kalman) State Estimation
  if (clampedRaw == 0.0f && currentMvMedian == 0.0f) {
    kalX = 0.0f;                             // Instant zero-reset when sensor is covered
    hybX = 0.0f;
  } else if (!filterInitialized) {
    kalX = clampedRaw;
    hybX = currentMvMedian;
    filterInitialized = true;
  } else {
    // Pure 1D Kalman update on raw signal
    kalP += 0.05f;
    float kG = kalP / (kalP + 1.0f);
    kalX += kG * (clampedRaw - kalX);
    kalP = (1.0f - kG) * kalP;

    // Hybrid Kalman update on spike-free median signal
    hybP += 0.05f;
    float hG = hybP / (hybP + 1.5f);
    hybX += hG * (currentMvMedian - hybX);
    hybP = (1.0f - hG) * hybP;
  }

  currentMvKalman = constrain(kalX, 0.0f, 3300.0f);
  currentMvHybrid = constrain(hybX, 0.0f, 3300.0f);

  // Enforce clean zero baseline if filtered output falls under 50 mV
  if (currentMvHybrid < 50.0f) {
    currentMvHybrid = 0.0f;
    currentMvRaw    = 0.0f;
    currentMvMedian = 0.0f;
    currentMvKalman = 0.0f;
  }

  // Compute final calibrated UVI, risk category, and current Wi-Fi RSSI
  currentUvIndex = calculateFullScaleUVI(currentMvHybrid);
  currentRisk    = getRiskCategory(currentUvIndex);
  currentRssi    = WiFi.RSSI();
}

// =========================================================================
// 9. LOCAL REST API ENDPOINT (/data)
// Purpose: Serves real-time telemetry as CORS-enabled JSON over local Wi-Fi
//          so the Flutter mobile app can poll directly via the ESP32's IP.
// =========================================================================
void handleJsonData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");

  JsonDocument doc;
  doc["seq"]                = packetSeq;
  doc["device_id"]          = DEVICE_ID;
  doc["mv_raw"]             = serialized(String(currentMvRaw, 2));
  doc["mv_median"]          = serialized(String(currentMvMedian, 2));
  doc["mv_kalman"]          = serialized(String(currentMvKalman, 2));
  doc["mv_hybrid"]          = serialized(String(currentMvHybrid, 2));
  doc["uv_index_estimated"] = serialized(String(currentUvIndex, 3));
  doc["risk_category"]      = currentRisk;
  doc["rssi_dbm"]           = currentRssi;
  doc["ip"]                 = WiFi.localIP().toString();

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

// =========================================================================
// 10. WI-FI STATION CONNECTION & IP DISPLAY ROUTINE
// Purpose: Connects the ESP32-C3 to the configured router in Station mode,
//          sets RF TX power to 8.5 dBm to minimize ADC supply ripple, and 
//          displays the assigned local IP address on the OLED screen.
// =========================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);        // Reduces RF current spikes on 3.3V rail
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Connecting Wi-Fi...");
    display.print("SSID: ");
    display.println(WIFI_SSID);
    display.display();
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ESP32 Connected!");
  display.setCursor(0, 16);
  display.println("IP Address:");
  display.setCursor(0, 32);
  display.println(WiFi.localIP().toString());
  display.display();
  delay(2000);
}

// =========================================================================
// 11. INFLUXDB CLOUD TIME-SERIES DATABASE LOGGER
// Purpose: Sends Line Protocol telemetry over HTTPS every 2.5 seconds (every 
//          5th sample) to InfluxDB Cloud for Grafana and historical analytics.
// =========================================================================
void publishToInfluxDb() {
  if (WiFi.status() != WL_CONNECTED) return;
  static int cloudSyncCounter = 0;
  cloudSyncCounter++;
  if (cloudSyncCounter < 5) return;          // Throttle cloud writes to once per 2.5s
  cloudSyncCounter = 0;

  NetworkClientSecure client;
  client.setInsecure();                      // Skip root CA verification for fast TLS
  HTTPClient https;

  if (https.begin(client, INFLUXDB_URL)) {
    https.addHeader("Authorization", "Token " + String(INFLUXDB_TOKEN));
    https.addHeader("Content-Type", "text/plain; charset=utf-8");

    String lineProtocol = String("uv_measurement,device=") + DEVICE_ID + " ";
    lineProtocol += "voltage_raw_mv=" + String(currentMvRaw, 2) + ",";
    lineProtocol += "voltage_hybrid_mv=" + String(currentMvHybrid, 2) + ",";
    lineProtocol += "uv_index_estimated=" + String(currentUvIndex, 3);

    https.POST(lineProtocol);
    https.end();
  }
}

// =========================================================================
// 12. SYSTEM INITIALIZATION (SETUP)
// Purpose: Initializes Serial debug output, configures 12-bit ADC resolution 
//          with 11 dB attenuation, starts I2C OLED, connects Wi-Fi, and 
//          launches the local HTTP server and HiveMQ MQTT client.
// =========================================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);                           // 12-bit precision (0–4095)
  analogSetPinAttenuation(UV_SENSOR_PIN, ADC_11db);   // Full 0–3.3V input range

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  connectWiFi();

  server.on("/data", HTTP_GET, handleJsonData);
  server.on("/data", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(204);
  });
  server.begin();

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setBufferSize(512);
}

// =========================================================================
// 13. MAIN REAL-TIME EXECUTION LOOP
// Purpose: Services incoming HTTP requests, maintains MQTT & Wi-Fi links, 
//          samples and filters UV data every 500 ms, publishes to MQTT & 
//          InfluxDB, and refreshes the local SSD1306 OLED status readout.
// =========================================================================
void loop() {
  server.handleClient();                     // Process incoming HTTP /data requests

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();                           // Auto-reconnect if Wi-Fi drops
  }

  if (!mqttClient.connected()) {
    String clientId = String(DEVICE_ID) + "_" + String(random(0xffff), HEX);
    mqttClient.connect(clientId.c_str());    // Reconnect to HiveMQ MQTT broker
  }
  mqttClient.loop();                         // Maintain MQTT keep-alive packets

  unsigned long now = millis();
  if (now - lastSampleMillis >= SAMPLE_INTERVAL_MS) {
    lastSampleMillis = now;
    packetSeq++;

    // Step 1: Acquire ADC reading and run 3-tier DSP filters
    updateFiltersAndSensors();

    // Step 2: Serialize and broadcast live JSON packet to HiveMQ MQTT
    JsonDocument doc;
    doc["seq"]                = packetSeq;
    doc["device_id"]          = DEVICE_ID;
    doc["mv_hybrid"]          = serialized(String(currentMvHybrid, 2));
    doc["uv_index_estimated"] = serialized(String(currentUvIndex, 3));
    doc["risk_category"]      = currentRisk;
    doc["rssi_dbm"]           = currentRssi;

    char jsonBuffer[384];
    serializeJson(doc, jsonBuffer);
    mqttClient.publish(MQTT_TOPIC, jsonBuffer);

    // Step 3: Log telemetry to InfluxDB Cloud
    publishToInfluxDb();

    // Step 4: Refresh SSD1306 OLED screen with IP, mV, UVI, Risk, and RSSI
    String oledRisk = getShortOledRisk(currentUvIndex);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.printf("IP:%s\n", WiFi.localIP().toString().c_str());
    display.println("--------------------");
    display.printf("Hyb mV: %.2f\n", currentMvHybrid);
    display.printf("UV Idx:  %.2f (%s)\n", currentUvIndex, oledRisk.c_str());
    display.printf("RSSI:    %d dBm (#%lu)\n", currentRssi, packetSeq);
    display.display();
  }
}