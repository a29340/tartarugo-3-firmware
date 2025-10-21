#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <NimBLEAdvertisedDevice.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <WiFi.h>
#include <time.h>
#include <WiFiCredentials.h>
#include "NimBLEBeacon.h"
#include "NimBLEEddystoneTLM.h"
#include "esp_camera.h"
#include "motor-utils.h"
#include "time-utils.h"
#include "stream-utils.h"

portMUX_TYPE rssiMux = portMUX_INITIALIZER_UNLOCKED;

Preferences prefs;

// Camera settings
// AI Thinker ESP32-CAM pin definition
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

// 4 for flash led or 33 for normal led
#define LED_GPIO_NUM 4

// Time settings

const char *ntpServer = "pool.ntp.org";
constexpr long gmtOffset_sec = 3600;      // adjust to your timezone
constexpr int daylightOffset_sec = 3600;  // adjust for DST

// WiFi and server config

bool isConnected = false;
bool isServerInitialized = false;
unsigned long lastExecution = 0;
unsigned long period = 250;
int8_t wifiOffCounter = 0;
int8_t healthCounter = 0;

const char *wifiStatusName[] = {"WL_IDLE_STATUS",    "WL_NO_SSID_AVAIL",
                                "WL_SCAN_COMPLETED", "WL_CONNECTED",
                                "WL_CONNECT_FAILED", "WL_CONNECTION_LOST",
                                "WL_DISCONNECTED"};

AsyncWebServer server(80);

// beacon MAC
String otherBeaconMAC = "ef:3b:94:f0:f4:d7";   // Milo
String targetBeaconMAC = "c2:e9:b3:bf:3c:33";  // Nina
int openBeaconThresholdRSSI = -60;
int closeBeaconThresholdRSSI = -73;

NimBLEScan *pBLEScan;
// Servo PIN
constexpr int FLASH_PIN = 4;



// === RSSI smoothing ===
#define RSSI_SAMPLES 8

#define MAX_NAME_LEN 32
#define MAX_MAC_LEN 18

struct CatItem {
    char name[MAX_NAME_LEN];
    char mac[MAX_MAC_LEN];
    bool canFeed;
};
#define CATS_MAX_SIZE 5
CatItem cats[CATS_MAX_SIZE];
unsigned int catsSize = 0;

int rssiBuffers[CATS_MAX_SIZE][RSSI_SAMPLES];
bool bufferFilled[CATS_MAX_SIZE];
int rssiIndexes[CATS_MAX_SIZE];
int lastAvgRSSI[CATS_MAX_SIZE];
int lastSeenTimestamp[CATS_MAX_SIZE];



#define SCHEDULE_MAX_SIZE 10
ScheduleItem schedule[SCHEDULE_MAX_SIZE];
unsigned int scheduleSize = 0;



void saveLastFeed() {
    getLocalTime(&lastFeedTime);
    // Convert struct tm → time_t
    time_t t = mktime(&lastFeedTime);
    prefs.begin("catfeeder", false);
    prefs.putLong64("lastFeed", (int64_t)t);
    prefs.putLong("lastFeedAmount", lastFeedAmount);
    prefs.end();
}

void saveScheduleBlob() {
    prefs.begin("catfeeder", false);
    prefs.putBytes("schedule", schedule, scheduleSize * sizeof(ScheduleItem));
    prefs.end();
}

void saveSettings() {
    prefs.begin("catfeeder", false);
    prefs.putBytes("cats", cats, catsSize * sizeof(CatItem));
    prefs.putInt("openBeaconRSSI", openBeaconThresholdRSSI);
    prefs.putInt("closeBeaconRSSI", closeBeaconThresholdRSSI);
    prefs.end();
}



int averageRSSI(int ci, int rssi) {
    portENTER_CRITICAL(&rssiMux);
    int rssiIndex = rssiIndexes[ci];
    rssiBuffers[ci][rssiIndex] = rssi;
    rssiIndex = (rssiIndex + 1) % RSSI_SAMPLES;
    rssiIndexes[ci] = rssiIndex;
    if (rssiIndex == 0) bufferFilled[ci] = true;
    int count = bufferFilled[ci] ? RSSI_SAMPLES : rssiIndexes[ci];
    if (count == 0) {
        portEXIT_CRITICAL(&rssiMux);
        return -999;  // no samples yet
    }
    long sum = 0;
    for (int i = 0; i < count; i++) sum += rssiBuffers[ci][i];
    lastSeenTimestamp[ci] = millis();
    portEXIT_CRITICAL(&rssiMux);
    return sum / count;
}



String getStatus() {
    JsonDocument doc;
    const JsonObject status = doc.to<JsonObject>();
    status["wifiSignal"] = WiFi.RSSI();
    status["isLidOpen"] = lidOpen;
    status["isAutoMode"] = !lidOverride;
    status["lastFeedTime"] =
        lastFeedAmount == 0 ? "never" : printDateTime(&lastFeedTime).c_str();
    status["lastFeedAmount"] = lastFeedAmount;
    status["lastStartupTime"] =
        timeIsSet ? printDateTime(&lastStartTime).c_str() : "unknown";
    const JsonArray catsJson = status["cats"].to<JsonArray>();

    for (size_t i = 0; i < catsSize; i++) {
        auto cat = catsJson.add<JsonObject>();
        cat["name"] = cats[i].name;
        cat["rssi"] = lastAvgRSSI[i];
        cat["canFeed"] = cats[i].canFeed;
        cat["mac"] = cats[i].mac;
        cat["lastSeen"] = lastSeenTimestamp[i];
    }
    String response;
    serializeJson(doc, response);
    return response;
}

String getStatusProm() {
    auto out = String(
        "# HELP feeder_cat_rssi rssi by cat\n# TYPE feeder_cat_rssi gauge\n");
    for (size_t i = 0; i < catsSize; i++) {
        out = out + String("feeder_cat_rssi{name=\"" + String(cats[i].name) +
                           "\"} " + String(lastAvgRSSI[i]) + "\n");
    }
    out = out + String(
                    "# HELP feeder_wifi_rssi wifi signal strength\n# TYPE "
                    "feeder_wifi_rssi gauge\n");
    out = out + String("feeder_wifi_rssi " + String(WiFi.RSSI()) + "\n");
    out = out + String(
                    "# HELP feeder_is_open indicates if lid is open or not\n# "
                    "TYPE feeder_is_open counter\n");
    out = out + String("feeder_is_open " + String(lidOpen) + "\n");
    out = out + String(
                    "# HELP feeder_threshold open or close rssi threshold\n# "
                    "TYPE feeder_threshold gauge\n");
    out = out + String("feeder_threshold{status=\"open\"} " +
                       String(openBeaconThresholdRSSI) + "\n");
    out = out + String("feeder_threshold{status=\"close\"} " +
                       String(closeBeaconThresholdRSSI) + "\n");
    return out;
}

class scanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
        const std::string mac = advertisedDevice->getAddress().toString();
        // update RSSI
        for (size_t ci = 0; ci < catsSize; ci++) {
            const CatItem cat = cats[ci];
            if (strcmp(mac.c_str(), cat.mac) == 0) {
                const int rssi = advertisedDevice->getRSSI();
                lastAvgRSSI[ci] = averageRSSI(ci, rssi);
            }
        }
    }
} scanCallbacks;

void initialiseWebServer() {
    Serial.println("Initializing WebServer");
    // --- GLOBAL CORS HOOK ---
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods",
                                         "GET, POST, OPTIONS, PUT, DELETE");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                         "Content-Type, Authorization");

    server.on("/api/flash/on", HTTP_POST, [](AsyncWebServerRequest *request) {
        digitalWrite(FLASH_PIN, HIGH);
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/flash/off", HTTP_POST, [](AsyncWebServerRequest *request) {
        digitalWrite(FLASH_PIN, LOW);
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/lid/open", HTTP_POST, [](AsyncWebServerRequest *request) {
        lidOverride = true;
        openLid();
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/lid/close", HTTP_POST, [](AsyncWebServerRequest *request) {
        lidOverride = true;
        closeLid();
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/lid/auto", HTTP_POST, [](AsyncWebServerRequest *request) {
        lidOverride = false;
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/status/prometheus", HTTP_GET,
              [](AsyncWebServerRequest *request) {
                  AsyncWebServerResponse *response = request->beginResponse(
                      200, "application/json", getStatusProm());
                  healthCounter = 0;
                  request->send(response);
              });
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncWebServerResponse *response =
            request->beginResponse(200, "application/json", getStatus());
        request->send(response);
    });
    server.on("/api/feed", HTTP_POST, [](AsyncWebServerRequest *request) {
        AsyncWebParameter *amountParam =
            request->getParam("amount", false, false);
        int amount = amountParam->value().toInt();
        feedAmount(amount);
        saveLastFeed();
        request->send(200, "application/json", getStatus());
    });
    server.on(
        "/api/schedule", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
           size_t index, size_t total) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                request->send(400, "application/json",
                              "{\"error\":\"Invalid JSON\"}");
                return;
            }
            JsonArray arr = doc.as<JsonArray>();
            scheduleSize = min((size_t)arr.size(), (size_t)SCHEDULE_MAX_SIZE);

            for (size_t i = 0; i < scheduleSize; i++) {
                schedule[i].hour = arr[i]["hour"] | 0;
                schedule[i].minute = arr[i]["minute"] | 0;
                schedule[i].amount = arr[i]["amount"] | 0;
            }
            saveScheduleBlob();
            request->send(201, "application/json", "{\"status\":\"ok\"}");
        });

    server.on(
        "/api/settings", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
           size_t index, size_t total) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err) {
                request->send(400, "application/json",
                              "{\"error\":\"Invalid JSON\"}");
                return;
            }
            JsonObject settings = doc.as<JsonObject>();
            JsonArray catsJsonArray = settings["cats"];
            catsSize = min((size_t)catsJsonArray.size(), (size_t)CATS_MAX_SIZE);
            for (size_t i = 0; i < catsSize; i++) {
                strlcpy(cats[i].name, catsJsonArray[i]["name"] | "",
                        MAX_NAME_LEN);
                strlcpy(cats[i].mac, catsJsonArray[i]["mac"] | "", MAX_MAC_LEN);
                cats[i].canFeed = catsJsonArray[i]["canFeed"] | false;
            }
            openBeaconThresholdRSSI =
                settings["openBeaconThresholdRSSI"] | openBeaconThresholdRSSI;
            closeBeaconThresholdRSSI =
                settings["closeBeaconThresholdRSSI"] | closeBeaconThresholdRSSI;
            saveSettings();
            request->send(201, "application/json", "{\"status\":\"ok\"}");
        });
    server.on("/api/schedule", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();

        for (size_t i = 0; i < scheduleSize; i++) {
            JsonObject obj = arr.add<JsonObject>();
            obj["hour"] = schedule[i].hour;
            obj["minute"] = schedule[i].minute;
            obj["amount"] = schedule[i].amount;
        }
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
        JsonDocument doc;
        JsonObject settings = doc.to<JsonObject>();
        settings["openBeaconThresholdRSSI"] = openBeaconThresholdRSSI;
        settings["closeBeaconThresholdRSSI"] = closeBeaconThresholdRSSI;
        JsonArray catsJson = settings["cats"].to<JsonArray>();
        for (size_t i = 0; i < catsSize; i++) {
            JsonObject cat = catsJson.add<JsonObject>();
            cat["name"] = cats[i].name;
            cat["mac"] = cats[i].mac;
            cat["canFeed"] = cats[i].canFeed;
        }
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    // Global OPTIONS handler (for preflight requests)
    server.onNotFound([](AsyncWebServerRequest *request) {
        if (request->method() == HTTP_OPTIONS) {
            request->send(204);
        } else {
            request->send(404, "text/plain", "Not found");
        }
    });

    server.on("/api/snapshot", HTTP_GET, [](AsyncWebServerRequest *request) {
        camera_fb_t *fb = esp_camera_fb_get();  // capture frame
        if (!fb) {
            request->send(500, "text/plain", "Camera capture failed");
            return;
        }

        AsyncWebServerResponse *response =
            request->beginResponse_P(200, "image/jpeg", fb->buf, fb->len);
        response->addHeader("Content-Disposition",
                            "inline; filename=capture.jpg");
        request->send(response);
        esp_camera_fb_return(fb);  // return buffer to driver
    });

    // MJPEG stream
    server.on("/api/stream", HTTP_GET, [](AsyncWebServerRequest *request) {
        handleStream(request);
    });
    server.begin();
}

void loadSettings() {
    prefs.begin("catfeeder", true);
    const time_t t = static_cast<time_t>(prefs.getLong64("lastFeed", 0));
    if (t != 0) {
        localtime_r(&t, &lastFeedTime);
    }
    lastFeedAmount = prefs.getLong("lastFeedAmount", 0);
    const size_t len = prefs.getBytes("schedule", &schedule, SCHEDULE_MAX_SIZE * sizeof(ScheduleItem));
    scheduleSize = len / sizeof(ScheduleItem);
    for (int i = 0; i < scheduleSize; i++) {
        Serial.printf("Scheduled feeding: %d:%d amount: %d\n", schedule[i].hour, schedule[i].minute, schedule[i].amount);
    }
    openBeaconThresholdRSSI = prefs.getInt("openBeaconRSSI", openBeaconThresholdRSSI);
    closeBeaconThresholdRSSI = prefs.getInt("closeBeaconRSSI", closeBeaconThresholdRSSI);
    Serial.printf("Configured thresholds - open %d - close: %d \n", openBeaconThresholdRSSI, closeBeaconThresholdRSSI);

    const size_t len2 = prefs.getBytes("cats", &cats, CATS_MAX_SIZE * sizeof(CatItem));
    catsSize = len2 / sizeof(CatItem);

    for (size_t i = 0; i < catsSize; i++) {
        Serial.printf("Configured cats: %s - mac: %s - canFeed: %s\n",
                      cats[i].name, cats[i].mac,
                      cats[i].canFeed ? "true" : "false");
    }

    prefs.end();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Setup started!");
    pinMode(FLASH_PIN, OUTPUT);
    // Init Camera
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG;  // for streaming
    // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 15;
    config.fb_count = 4;
    config.grab_mode = CAMERA_GRAB_LATEST;

    // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
    //                      for larger pre-allocated frame buffer.

    esp_err_t err = esp_camera_init(&config);
    for (size_t i = 0; i < 3; i++) {
        if (err != ESP_OK) {
            Serial.printf("Camera init failed with error 0x%x\n", err);
            delay(500);
            err = esp_camera_init(&config);
        } else {
            break;
        }
    }

    loadSettings();

    setupMotors();

    // Init BLE
    NimBLEDevice::init("Beacon-scanner");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setScanCallbacks(&scanCallbacks, true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->setMaxResults(0);
    pBLEScan->start(0, false, true);
    Serial.println("BLE scanning started...");

    // Init time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Setup WiFi
    Serial.print("Init WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    for (int i = 0; i < CATS_MAX_SIZE; i++) {
        lastAvgRSSI[i] = -99;
        lastSeenTimestamp[i] = millis();
    }
    Serial.println("Setup done!");
}

void afterWifiConnected() {
    Serial.println("Init Webserver");
    initialiseWebServer();
    Serial.println("Init OTA");
    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else {  // U_SPIFFS
                type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) {
                Serial.println("Auth Failed");
            } else if (error == OTA_BEGIN_ERROR) {
                Serial.println("Begin Failed");
            } else if (error == OTA_CONNECT_ERROR) {
                Serial.println("Connect Failed");
            } else if (error == OTA_RECEIVE_ERROR) {
                Serial.println("Receive Failed");
            } else if (error == OTA_END_ERROR) {
                Serial.println("End Failed");
            }
        });

    ArduinoOTA.begin();
    isServerInitialized = true;
}

void checkWiFiAndPrint()
{
    const wl_status_t wifiStatus = WiFi.status();
    isConnected = wifiStatus == WL_CONNECTED;
    if (isConnected) {
        if (!isServerInitialized) {
            afterWifiConnected();
            Serial.printf("Address: %s\n", WiFi.localIP().toString().c_str());
        }
        if (!timeIsSet && getLocalTime(&timeinfo)) {
            Serial.println("Acquired NTP time");
            getLocalTime(&lastStartTime);
            timeIsSet = true;
        }
    } else {
        Serial.printf("Wifi status: %s\n", wifiStatusName[wifiStatus]);
        wifiOffCounter++;
        // reconnect after 5 seconds of off time
        if (wifiOffCounter >= 20)
        {
            ESP.restart();
        }
    }
}

void checkLid(const unsigned long now) {
    int targetBiggerRSSI = -99;
    int otherCatBiggerRSSI = -99;
    for (size_t i = 0; i < catsSize; i++) {
        const int avg = lastAvgRSSI[i];
        if (cats[i].canFeed) {
            if (targetBiggerRSSI < avg) {
                targetBiggerRSSI = avg;
            }
        } else {
            if (otherCatBiggerRSSI < avg) {
                otherCatBiggerRSSI = avg;
            }
        }
    }

    if (!lidOverride && !lidMotion.active) {
        const bool targetCatIsClose = targetBiggerRSSI > openBeaconThresholdRSSI;
        const bool targetCatIsNotClose = targetBiggerRSSI < closeBeaconThresholdRSSI;
        const bool anotherCatIsCloserThanTarget = otherCatBiggerRSSI > openBeaconThresholdRSSI && otherCatBiggerRSSI > targetBiggerRSSI;

        if ((targetCatIsNotClose || anotherCatIsCloserThanTarget) &&
            ((now - lastOpen) >= 3000) && lidOpen) {
            closeLid();
            return;
        }

        if (targetCatIsClose && !anotherCatIsCloserThanTarget && !lidOpen) {
            openLid();
        }
    }

    if (now - lastClosed > 10000 && !lidOpen && lidMotion.attached) {
        lidMotion.servo->detach();
        lidMotion.attached = false;
    }
}

void checkHealth() {
    healthCounter++;
    if (healthCounter >= 20)
    {
        ESP.restart();
    }
}

void checkLastSeen(const unsigned long now) {
    for (int ci = 0; ci < CATS_MAX_SIZE; ci++) {
        if (now - lastSeenTimestamp[ci] >= 3000) {
            lastAvgRSSI[ci] = averageRSSI(ci, -99);
        }
    }
}

void periodic() {
    const unsigned long now = millis();
    if (now - lastExecution > period) {
        checkWiFiAndPrint();
        checkLid(now);
        checkHealth();
        checkLastSeen(now);
        lastExecution = now;
    }
}

void runSchedule() {
    for (int i = 0; i < scheduleSize; i++) {
        tm nowTm;
        if (timeIsSet && getLocalTime(&nowTm)) {
            if (schedule[i].hour == nowTm.tm_hour &&
                schedule[i].minute == nowTm.tm_min && nowTm.tm_sec == 0) {
                Serial.printf("Feeding time! %d\n", schedule[i].amount);
                feedAmount(schedule[i].amount);
                saveLastFeed();
                delay(2000);
            }
        }
    }
}



void loop() {
    periodic();

    // Update servo
    updateSmoothMove();

    // Update stepper
    updateStepper();

    // Check schedule
    runSchedule();

    ArduinoOTA.handle();
}