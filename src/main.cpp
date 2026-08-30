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
#include "generated/openapi.h"
#include "motor-utils.h"
#include "time-utils.h"

void mqttInit();
void mqttUpdate();
void mqttPublishState();

portMUX_TYPE rssiMux = portMUX_INITIALIZER_UNLOCKED;

SemaphoreHandle_t nvsMutex = NULL;

Preferences prefs;

// Time settings

const char* ntpServer = "pool.ntp.org";
constexpr long gmtOffset_sec = 3600; // adjust to your timezone
constexpr int daylightOffset_sec = 3600; // adjust for DST

// WiFi and server config

bool isConnected = false;
bool isServerInitialized = false;
unsigned long lastExecution = 0;
unsigned long lastExecution5s = 0;
int8_t healthCounter = 0;

const char* wifiStatusName[] = {
    "WL_IDLE_STATUS", "WL_NO_SSID_AVAIL",
    "WL_SCAN_COMPLETED", "WL_CONNECTED",
    "WL_CONNECT_FAILED", "WL_CONNECTION_LOST",
    "WL_DISCONNECTED"
};

AsyncWebServer server(80);

// beacon MAC
String otherBeaconMAC = "ef:3b:94:f0:f4:d7"; // Milo
String targetBeaconMAC = "c2:e9:b3:bf:3c:33"; // Nina
int openBeaconThresholdRSSI = -60;
int closeBeaconThresholdRSSI = -73;
// amount used by the HA "Feed amount" number / "Feed" button, persisted in NVS
int feedAmountSetting = 250;

NimBLEScan* pBLEScan;

// === RSSI smoothing ===
#define RSSI_SAMPLES 8

#define MAX_NAME_LEN 32
#define MAX_MAC_LEN 18

struct CatItem
{
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

#define MAX_LOGS 50
#define MAX_LOG_LEN 128

void logEvent(const char* msg) {
    xSemaphoreTake(nvsMutex, portMAX_DELAY);
    prefs.begin("logs", false);
    uint8_t idx = prefs.getUChar("idx", 0);
    char key[8];
    snprintf(key, sizeof(key), "l%02d", idx);
    prefs.putString(key, msg);
    idx = (idx + 1) % MAX_LOGS;
    prefs.putUChar("idx", idx);
    prefs.end();
    xSemaphoreGive(nvsMutex);
}

void saveResetReason() {
    char buf[128];
    const esp_reset_reason_t reason = esp_reset_reason();
    String r;
    switch (reason)
    {
    case ESP_RST_POWERON:
        r = "Power on";
        break;
    case ESP_RST_EXT:
        r = "External reset";
        break;
    case ESP_RST_SW:
        r = "Software reset";
        break;
    case ESP_RST_PANIC:
        r = "Panic / crash";
        break;
    case ESP_RST_INT_WDT:
        r = "Interrupt watchdog";
        break;
    case ESP_RST_TASK_WDT:
        r = "Task watchdog";
        break;
    case ESP_RST_BROWNOUT:
        r = "Brownout";
        break;
    default:
        r = "Unknown";
        break;
    }
    snprintf(buf, sizeof(buf),
             "Reboot reason=%s", r.c_str());
    logEvent(buf);
}

void saveLastFeed()
{
    getLocalTime(&lastFeedTime);
    // Convert struct tm → time_t
    time_t t = mktime(&lastFeedTime);
    xSemaphoreTake(nvsMutex, portMAX_DELAY);
    prefs.begin("catfeeder", false);
    prefs.putLong64("lastFeed", (int64_t)t);
    prefs.putLong("lastFeedAmount", lastFeedAmount);
    prefs.end();
    xSemaphoreGive(nvsMutex);
}

void saveScheduleBlob()
{
    xSemaphoreTake(nvsMutex, portMAX_DELAY);
    prefs.begin("catfeeder", false);
    prefs.putUChar("scheduleVersion", SCHEDULE_BLOB_VERSION);
    prefs.putBytes("schedule", schedule, scheduleSize * sizeof(ScheduleItem));
    prefs.end();
    xSemaphoreGive(nvsMutex);
}

void saveSettings()
{
    xSemaphoreTake(nvsMutex, portMAX_DELAY);
    prefs.begin("catfeeder", false);
    prefs.putBytes("cats", cats, catsSize * sizeof(CatItem));
    prefs.putInt("openBeaconRSSI", openBeaconThresholdRSSI);
    prefs.putInt("closeBeaconRSSI", closeBeaconThresholdRSSI);
    prefs.putInt("feedAmount", feedAmountSetting);
    prefs.end();
    xSemaphoreGive(nvsMutex);
}


int averageRSSI(int ci, int rssi)
{
    portENTER_CRITICAL(&rssiMux);
    int rssiIndex = rssiIndexes[ci];
    rssiBuffers[ci][rssiIndex] = rssi;
    rssiIndex = (rssiIndex + 1) % RSSI_SAMPLES;
    rssiIndexes[ci] = rssiIndex;
    if (rssiIndex == 0) bufferFilled[ci] = true;
    int count = bufferFilled[ci] ? RSSI_SAMPLES : rssiIndexes[ci];
    if (count == 0)
    {
        portEXIT_CRITICAL(&rssiMux);
        return -999; // no samples yet
    }
    long sum = 0;
    for (int i = 0; i < count; i++) sum += rssiBuffers[ci][i];
    lastSeenTimestamp[ci] = millis();
    portEXIT_CRITICAL(&rssiMux);
    return sum / count;
}


String getStatus()
{
    JsonDocument doc;
    const JsonObject status = doc.to<JsonObject>();
    status["wifiSignal"] = WiFi.RSSI();
    status["isLidOpen"] = lidOpen;
    status["isLid2Open"] = lid2Open;
    status["isAutoMode"] = !lidOverride;
    status["lastFeedTime"] =
        lastFeedAmount == 0 ? "never" : printDateTime(&lastFeedTime).c_str();
    status["lastFeedAmount"] = lastFeedAmount;
    status["lastStartupTime"] =
        timeIsSet ? printDateTime(&lastStartTime).c_str() : "unknown";
    const JsonArray catsJson = status["cats"].to<JsonArray>();

    for (size_t i = 0; i < catsSize; i++)
    {
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

String getStatusProm()
{
    auto out = String(
        "# HELP feeder_cat_rssi rssi by cat\n# TYPE feeder_cat_rssi gauge\n");
    for (size_t i = 0; i < catsSize; i++)
    {
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
        "# HELP feeder_lid2_open indicates if the second lid is open or not\n# "
        "TYPE feeder_lid2_open gauge\n");
    out = out + String("feeder_lid2_open " + String(lid2Open) + "\n");
    out = out + String(
        "# HELP feeder_threshold open or close rssi threshold\n# "
        "TYPE feeder_threshold gauge\n");
    out = out + String("feeder_threshold{status=\"open\"} " +
        String(openBeaconThresholdRSSI) + "\n");
    out = out + String("feeder_threshold{status=\"close\"} " +
        String(closeBeaconThresholdRSSI) + "\n");
    return out;
}

class scanCallbacks : public NimBLEScanCallbacks
{
    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override
    {
        const std::string mac = advertisedDevice->getAddress().toString();
        // update RSSI
        for (size_t ci = 0; ci < catsSize; ci++)
        {
            const CatItem cat = cats[ci];
            if (strcmp(mac.c_str(), cat.mac) == 0)
            {
                const int rssi = advertisedDevice->getRSSI();
                lastAvgRSSI[ci] = averageRSSI(ci, rssi);
            }
        }
    }
} scanCallbacks;

void initialiseWebServer()
{
    Serial.println("Initializing WebServer");
    // --- GLOBAL CORS HOOK ---
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods",
                                         "GET, POST, OPTIONS, PUT, DELETE");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                         "Content-Type, Authorization");

    server.on("/openapi", HTTP_GET, [](AsyncWebServerRequest* request)
    {
        request->send_P(200, "application/yaml",
                        reinterpret_cast<const uint8_t*>(openapiSpec),
                        sizeof(openapiSpec) - 1);
    });
    server.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest* request)
    {
        xSemaphoreTake(nvsMutex, portMAX_DELAY);
        prefs.begin("logs", true);
        const uint8_t idx = prefs.getUChar("idx", 0);

        JsonDocument doc;
        const JsonObject obj = doc.to<JsonObject>();
        obj["idx"] = idx;
        obj["logs"] = doc.add<JsonObject>();

        for (int i = idx; i < MAX_LOGS + idx; i++)
        {
            char key[8];
            snprintf(key, sizeof(key), "l%02d", (idx + i) % MAX_LOGS);
            String msg = prefs.getString(key, "");
            if (msg.length()) obj["logs"][String((idx + i) % MAX_LOGS)] = msg;
        }
        prefs.end();
        xSemaphoreGive(nvsMutex);
        String out;
        serializeJson(doc, out);
        request->send(200, "application/json", out);
    });
    server.on("/api/lid/open", HTTP_POST, [](AsyncWebServerRequest* request)
    {
        lidOverride = true;
        const AsyncWebParameter* lidParam = request->getParam("lid");
        if (lidParam && lidParam->value() == "lid2") {
            openLid(LID_2);
        } else {
            openLid(LID_1);
        }
        mqttPublishState();
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/lid/close", HTTP_POST, [](AsyncWebServerRequest* request)
    {
        const AsyncWebParameter* lidParam = request->getParam("lid");
        if (lidParam && lidParam->value() == "lid2")
        {
            closeLid(LID_2);
        }
        else
        {
            lidOverride = true;
            closeLid(LID_1);
        }
        mqttPublishState();
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/lid/auto", HTTP_POST, [](AsyncWebServerRequest* request)
    {
        lidOverride = false;
        mqttPublishState();
        request->send(200, "application/json", getStatus());
    });
    server.on("/api/status/prometheus", HTTP_GET,
              [](AsyncWebServerRequest* request)
              {
                  AsyncWebServerResponse* response = request->beginResponse(
                      200, "application/json", getStatusProm());
                  healthCounter = 0;
                  request->send(response);
              });
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request)
    {
        AsyncWebServerResponse* response =
            request->beginResponse(200, "application/json", getStatus());
        request->send(response);
    });
    server.on("/api/feed", HTTP_POST, [](AsyncWebServerRequest* request)
    {
        AsyncWebParameter* amountParam =
            request->getParam("amount", false, false);
        int amount = amountParam->value().toInt();
        feedAmount(amount);
        saveLastFeed();
        mqttPublishState();
        request->send(200, "application/json", getStatus());
    });
    server.on(
        "/api/schedule", HTTP_POST, [](AsyncWebServerRequest* request)
        {
        }, NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len,
           size_t index, size_t total)
        {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err)
            {
                request->send(400, "application/json",
                              "{\"error\":\"Invalid JSON\"}");
                return;
            }
            JsonArray arr = doc.as<JsonArray>();
            scheduleSize = min((size_t)arr.size(), (size_t)SCHEDULE_MAX_SIZE);

            for (size_t i = 0; i < scheduleSize; i++)
            {
                schedule[i].hour = arr[i]["hour"] | 0;
                schedule[i].minute = arr[i]["minute"] | 0;
                schedule[i].amount = arr[i]["amount"] | 0;
                schedule[i].isLid2 = arr[i]["isLid2"] | false;
                schedule[i].lid2Open = arr[i]["lid2Open"] | true;
            }
            saveScheduleBlob();
            request->send(201, "application/json", "{\"status\":\"ok\"}");
        });

    server.on(
        "/api/settings", HTTP_POST, [](AsyncWebServerRequest* request)
        {
        }, NULL,
        [](AsyncWebServerRequest* request, uint8_t* data, size_t len,
           size_t index, size_t total)
        {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, data, len);
            if (err)
            {
                request->send(400, "application/json",
                              "{\"error\":\"Invalid JSON\"}");
                return;
            }
            JsonObject settings = doc.as<JsonObject>();
            JsonArray catsJsonArray = settings["cats"];
            catsSize = min((size_t)catsJsonArray.size(), (size_t)CATS_MAX_SIZE);
            for (size_t i = 0; i < catsSize; i++)
            {
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
            mqttPublishState();
            request->send(201, "application/json", "{\"status\":\"ok\"}");
        });
    server.on("/api/schedule", HTTP_GET, [](AsyncWebServerRequest* request)
    {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();

        for (size_t i = 0; i < scheduleSize; i++)
        {
            JsonObject obj = arr.add<JsonObject>();
            obj["hour"] = schedule[i].hour;
            obj["minute"] = schedule[i].minute;
            obj["amount"] = schedule[i].amount;
            obj["isLid2"] = schedule[i].isLid2;
            obj["lid2Open"] = schedule[i].lid2Open;
        }
        String response;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
    });

    server.on("/api/settings", HTTP_GET, [](AsyncWebServerRequest* request)
    {
        JsonDocument doc;
        JsonObject settings = doc.to<JsonObject>();
        settings["openBeaconThresholdRSSI"] = openBeaconThresholdRSSI;
        settings["closeBeaconThresholdRSSI"] = closeBeaconThresholdRSSI;
        JsonArray catsJson = settings["cats"].to<JsonArray>();
        for (size_t i = 0; i < catsSize; i++)
        {
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
    server.onNotFound([](AsyncWebServerRequest* request)
    {
        if (request->method() == HTTP_OPTIONS)
        {
            request->send(204);
        }
        else
        {
            request->send(404, "text/plain", "Not found");
        }
    });

    server.begin();
}

void loadSettings()
{
    xSemaphoreTake(nvsMutex, portMAX_DELAY);
    prefs.begin("catfeeder", true);
    const time_t t = static_cast<time_t>(prefs.getLong64("lastFeed", 0));
    if (t != 0)
    {
        localtime_r(&t, &lastFeedTime);
    }
    lastFeedAmount = prefs.getLong("lastFeedAmount", 0);
    const size_t len = prefs.getBytes("schedule", &schedule, SCHEDULE_MAX_SIZE * sizeof(ScheduleItem));
    if (prefs.getUChar("scheduleVersion", 0) != SCHEDULE_BLOB_VERSION ||
        len % sizeof(ScheduleItem) != 0)
    {
        scheduleSize = 0;
    }
    else
    {
        scheduleSize = len / sizeof(ScheduleItem);
    }
    for (int i = 0; i < scheduleSize; i++)
    {
        if (schedule[i].isLid2)
        {
            Serial.printf("Scheduled lid2 %s: %d:%d\n",
                          schedule[i].lid2Open ? "open" : "close",
                          schedule[i].hour, schedule[i].minute);
        }
        else
        {
            Serial.printf("Scheduled feeding: %d:%d amount: %d\n", schedule[i].hour, schedule[i].minute,
                          schedule[i].amount);
        }
    }
    openBeaconThresholdRSSI = prefs.getInt("openBeaconRSSI", openBeaconThresholdRSSI);
    closeBeaconThresholdRSSI = prefs.getInt("closeBeaconRSSI", closeBeaconThresholdRSSI);
    feedAmountSetting = prefs.getInt("feedAmount", feedAmountSetting);
    Serial.printf("Configured thresholds - open %d - close: %d \n", openBeaconThresholdRSSI, closeBeaconThresholdRSSI);

    const size_t len2 = prefs.getBytes("cats", &cats, CATS_MAX_SIZE * sizeof(CatItem));
    catsSize = len2 / sizeof(CatItem);

    for (size_t i = 0; i < catsSize; i++)
    {
        Serial.printf("Configured cats: %s - mac: %s - canFeed: %s\n",
                      cats[i].name, cats[i].mac,
                      cats[i].canFeed ? "true" : "false");
    }

    prefs.end();
    xSemaphoreGive(nvsMutex);
}

#include "mqtt-utils.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);
    nvsMutex = xSemaphoreCreateMutex();
    saveResetReason();
    Serial.println("Setup started!");
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
    for (int i = 0; i < CATS_MAX_SIZE; i++)
    {
        lastAvgRSSI[i] = -99;
        lastSeenTimestamp[i] = millis();
    }
    Serial.println("Setup done!");
}

void afterWifiConnected()
{
    Serial.println("Init Webserver");
    initialiseWebServer();
    Serial.println("Init OTA");
    ArduinoOTA
        .onStart([]()
        {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
            {
                type = "sketch";
            }
            else
            {
                // U_SPIFFS
                type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount
            // SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
        {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error)
        {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
            {
                Serial.println("Auth Failed");
            }
            else if (error == OTA_BEGIN_ERROR)
            {
                Serial.println("Begin Failed");
            }
            else if (error == OTA_CONNECT_ERROR)
            {
                Serial.println("Connect Failed");
            }
            else if (error == OTA_RECEIVE_ERROR)
            {
                Serial.println("Receive Failed");
            }
            else if (error == OTA_END_ERROR)
            {
                Serial.println("End Failed");
            }
        });

    ArduinoOTA.begin();
    mqttInit();
    isServerInitialized = true;
}

void checkWiFiAndPrint()
{
    const wl_status_t wifiStatus = WiFi.status();
    isConnected = wifiStatus == WL_CONNECTED;
    if (isConnected)
    {
        if (!isServerInitialized)
        {
            afterWifiConnected();
            Serial.printf("Address: %s\n", WiFi.localIP().toString().c_str());
        }
        if (!timeIsSet && getLocalTime(&timeinfo))
        {
            Serial.println("Acquired NTP time");
            getLocalTime(&lastStartTime);
            timeIsSet = true;
            logEvent(printDateTime(&lastStartTime).c_str());
        }
    }
}

void checkLid(const unsigned long now)
{
    int targetBiggerRSSI = -99;
    int otherCatBiggerRSSI = -99;
    for (size_t i = 0; i < catsSize; i++)
    {
        const int avg = lastAvgRSSI[i];
        if (cats[i].canFeed)
        {
            if (targetBiggerRSSI < avg)
            {
                targetBiggerRSSI = avg;
            }
        }
        else
        {
            if (otherCatBiggerRSSI < avg)
            {
                otherCatBiggerRSSI = avg;
            }
        }
    }

    if (!lidOverride && !lidMotion.active)
    {
        const bool targetCatIsClose = targetBiggerRSSI > openBeaconThresholdRSSI;
        const bool targetCatIsNotClose = targetBiggerRSSI < closeBeaconThresholdRSSI;
        const bool anotherCatIsCloserThanTarget = otherCatBiggerRSSI > openBeaconThresholdRSSI && otherCatBiggerRSSI >
            targetBiggerRSSI;
        if ((targetCatIsNotClose || anotherCatIsCloserThanTarget) &&
            now - lastOpen >= 3000)
        {
            if (lidOpen)
            {
                closeLid(LID_1);
            } else
            {
                if (anotherCatIsCloserThanTarget)
                {
                    // keepClosed();
                    // lastClosed = now;
                }
            }
            return;
        }

        if (targetCatIsClose && !anotherCatIsCloserThanTarget && !lidOpen)
        {
            openLid(LID_1);
            return;
        }
    }


    if (now - lastClosed > 10000 && !lidOpen && lidMotion.attached) {
        lidMotion.servo->detach();
        lidMotion.attached = false;
    }

    if (now - lastLid2Closed > 10000 && !lid2Open && lid2Motion.attached) {
        lid2Motion.servo->detach();
        lid2Motion.attached = false;
    }
}

void checkHealth() {
    healthCounter++;
    if (healthCounter >= 100)
    {
        ESP.restart();
    }
}

void checkLastSeen(const unsigned long now)
{
    for (int ci = 0; ci < CATS_MAX_SIZE; ci++)
    {
        if (now - lastSeenTimestamp[ci] >= 3000)
        {
            lastAvgRSSI[ci] = averageRSSI(ci, -99);
        }
    }
}

void everyPeriod(const unsigned int period)
{
    const unsigned long now = millis();
    if (now - lastExecution > period)
    {
        checkWiFiAndPrint();
        checkLid(now);
        // checkHealth();
        checkLastSeen(now);
        lastExecution = now;
    }
}

void runSchedule()
{
    for (int i = 0; i < scheduleSize; i++)
    {
        tm nowTm;
        if (timeIsSet && getLocalTime(&nowTm))
        {
            if (schedule[i].hour == nowTm.tm_hour &&
                schedule[i].minute == nowTm.tm_min && nowTm.tm_sec == 0)
            {
                if (schedule[i].isLid2)
                {
                    if (schedule[i].lid2Open)
                    {
                        Serial.println("Scheduled: open lid2!");
                        openLid(LID_2);
                    }
                    else
                    {
                        Serial.println("Scheduled: close lid2!");
                        closeLid(LID_2);
                    }
                    mqttPublishState();
                }
                else
                {
                    Serial.printf("Feeding time! %d\n", schedule[i].amount);
                    openLid(LID_2);
                    feedAmount(schedule[i].amount);
                    saveLastFeed();
                    delay(2000);
                    mqttPublishState();
                }
            }
        }
    }
}


void loop()
{
    everyPeriod(500);

    // Update servos
    updateSmoothMoves();

    // Update stepper
    updateStepper();

    // Check schedule
    runSchedule();

    // MQTT: reconnect if needed, process traffic, periodic state publish
    mqttUpdate();

    ArduinoOTA.handle();
}
