#include <AccelStepper.h>
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
const long gmtOffset_sec = 3600;      // adjust to your timezone
const int daylightOffset_sec = 3600;  // adjust for DST

struct tm timeinfo;
boolean timeIsSet = false;
struct tm lastFeedTime;
long lastFeedAmount = 0;
struct tm lastStartTime;

// WIFI and server config

bool isConnected = false;
bool isServerInitialized = false;
unsigned long lastLog = 0;
unsigned long printPeriodicLog = 5000;
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
const int servoPIN = 12;
const int SERVO_DURATION_TOLERANCE_MS = 4000;
const int FLASH_PIN = 4;
Servo lidServo;
// Stepper PIN
const int stepPIN1 = 13;
const int stepPIN2 = 14;
const int stepPIN3 = 15;
const int stepPIN4 = 2;
AccelStepper stepper(AccelStepper::FULL4WIRE, stepPIN1, stepPIN3, stepPIN2,
                     stepPIN4);

bool lidOpen = false;
bool lidOverride = false;
int openAngle = 185;
int closedAngle = 78;

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

int feedingCat = 0;

// === Timeout setup ===
unsigned long lastSeenTarget = 0;
unsigned long lastSeenOther = 0;
const unsigned int bufferTimeout = 10000;
unsigned long lastOpen = 0;
const unsigned int minOpenTime = 5000;

struct ScheduleItem {
    uint8_t hour;
    uint8_t minute;
    uint16_t amount;
};

#define SCHEDULE_MAX_SIZE 10
ScheduleItem schedule[SCHEDULE_MAX_SIZE];
unsigned int scheduleSize = 0;

struct SmoothServo {
    Servo *servo;
    int from;
    int to;
    int steps;
    int duration;
    unsigned long startTime;
    bool active;
} lidMotion;

void startSmoothMove(Servo &servo, int from, int to, int steps, int duration) {
    lidMotion.servo = &servo;
    lidMotion.from = from;
    lidMotion.to = to;
    lidMotion.steps = steps;
    lidMotion.duration = duration;
    lidMotion.startTime = millis();
    lidMotion.active = true;
    lidMotion.servo->attach(servoPIN, 500, 2400);
}

void updateSmoothMove() {
    if (!lidMotion.active) return;

    unsigned long now = millis();
    unsigned long elapsed = now - lidMotion.startTime;

    if (elapsed >=
        ((unsigned long)lidMotion.duration + SERVO_DURATION_TOLERANCE_MS)) {
        lidMotion.servo->detach();
        lidMotion.active = false;
        return;
    }

    if (elapsed >= (unsigned long)lidMotion.duration) {
        lidMotion.servo->write(lidMotion.to);  // final angle
        return;
    }

    float progress = (float)elapsed / lidMotion.duration;  // 0..1
    // cosine ease-in/out
    float factor = (1 - cos(progress * PI)) / 2;
    int angle = lidMotion.from + (lidMotion.to - lidMotion.from) * factor;

    lidMotion.servo->write(angle);
}

void openLid() {
    if (!lidOpen) {
        startSmoothMove(lidServo, closedAngle, openAngle, 50, 1000);
    }
    lidOpen = true;
    lastOpen = millis();
    Serial.println("Open lid!");
}

void closeLid() {
    if (lidOpen) {
        startSmoothMove(lidServo, openAngle, closedAngle, 100, 1000);
    }
    lidOpen = false;
    Serial.println("Close lid!");
}

void saveLastFeed() {
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

void feedAmount(int amount) {
    stepper.setCurrentPosition(0);
    stepper.moveTo(amount);
    stepper.enableOutputs();
    getLocalTime(&lastFeedTime);
    lastFeedAmount = amount;
    saveLastFeed();
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
    portEXIT_CRITICAL(&rssiMux);
    return sum / count;
}

String printDateTime(tm *dateTime) {
    // 2015-03-25T12:00:00Z
    char buffer[26];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%X", dateTime);
    return String(buffer);
}

String getStatus() {
    JsonDocument doc;
    JsonObject status = doc.to<JsonObject>();
    status["wifiSignal"] = WiFi.RSSI();
    status["isLidOpen"] = lidOpen;
    status["isAutoMode"] = !lidOverride;
    status["lastFeedTime"] =
        lastFeedAmount == 0 ? "never" : printDateTime(&lastFeedTime).c_str();
    status["lastFeedAmount"] = lastFeedAmount;
    status["lastStartupTime"] =
        timeIsSet ? printDateTime(&lastStartTime).c_str() : "unknown";
    JsonArray catsJson = status["cats"].to<JsonArray>();

    for (size_t i = 0; i < catsSize; i++) {
        JsonObject cat = catsJson.add<JsonObject>();
        cat["name"] = cats[i].name;
        cat["rssi"] = lastAvgRSSI[i];
        cat["canFeed"] = cats[i].canFeed;
        cat["mac"] = cats[i].mac;
    }
    String response;
    serializeJson(doc, response);
    return response;
}

String getStatusProm() {
    String out = String(
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
        std::string mac = advertisedDevice->getAddress().toString();
        // update RSSI
        for (size_t ci = 0; ci < catsSize; ci++) {
            CatItem cat = cats[ci];
            if (strcmp(mac.c_str(), cat.mac) == 0) {
                int rssi = advertisedDevice->getRSSI();
                // Serial.printf("RSSI: %d\n", rssi);
                lastAvgRSSI[ci] = averageRSSI(ci, rssi);
                // pBLEScan->clearResults();
            }
        }
    }
} scanCallbacks;

void addCORSHeaders(AsyncWebServerResponse *response) {
    response->addHeader("Access-Control-Allow-Origin", "*");
    response->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    response->addHeader("Access-Control-Allow-Headers", "Content-Type");
}

static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=frame";
static const char *_STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

class AsyncJpegStreamResponse : public AsyncAbstractResponse {
   private:
    camera_fb_t *fb;
    unsigned long lastFrame;

   public:
    AsyncJpegStreamResponse() {
        _callback = nullptr;
        _code = 200;
        _contentType = _STREAM_CONTENT_TYPE;
        _sendContentLength = false;
        _chunked = true;
        fb = nullptr;
        lastFrame = 0;
    }
    ~AsyncJpegStreamResponse() {
        if (fb) esp_camera_fb_return(fb);
    }
    bool _sourceValid() const override { return true; }

    size_t _fillBuffer(uint8_t *buf, size_t maxLen) override {
        // throttle frame rate a little if needed
        if (millis() - lastFrame < 100) {
            return RESPONSE_TRY_AGAIN;  // ask AsyncWebServer to try again
        }
        lastFrame = millis();

        fb = esp_camera_fb_get();
        if (!fb) {
            return RESPONSE_TRY_AGAIN;
        }

        // build the multipart headers
        char part[64];
        size_t len = snprintf(part, 64, _STREAM_PART, fb->len);
        size_t headerLen = strlen(_STREAM_BOUNDARY) + len;

        if (headerLen + fb->len > maxLen) {
            // too big for buffer → release frame and retry
            esp_camera_fb_return(fb);
            fb = nullptr;
            return RESPONSE_TRY_AGAIN;
        }

        memcpy(buf, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        memcpy(buf + strlen(_STREAM_BOUNDARY), part, len);
        memcpy(buf + headerLen, fb->buf, fb->len);

        size_t total = headerLen + fb->len;
        esp_camera_fb_return(fb);
        fb = nullptr;

        return total;
    }
};

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
        // AsyncWebServerResponse *response = request->beginResponse(
        //     STREAM_CONTENT_TYPE, 0,
        //     [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
        //         camera_fb_t *fb = esp_camera_fb_get();
        //         if (!fb) {
        //             return 0;
        //         }

        //         // Header for frame
        //         char part_buf[64];
        //         size_t header_len =
        //             snprintf(part_buf, 64, STREAM_PART, fb->len);

        //         // Total size needed
        //         size_t total_len =
        //             strlen(STREAM_BOUNDARY) + header_len + fb->len;

        //         if (maxLen < total_len) {
        //             esp_camera_fb_return(fb);
        //             return 0;
        //         }

        //         // Copy boundary
        //         memcpy(buffer, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        //         size_t used = strlen(STREAM_BOUNDARY);

        //         // Copy headers
        //         memcpy(buffer + used, part_buf, header_len);
        //         used += header_len;

        //         // Copy JPEG data
        //         memcpy(buffer + used, fb->buf, fb->len);
        //         used += fb->len;

        //         esp_camera_fb_return(fb);
        //         return used;
        //     });

        // response->addHeader("Access-Control-Allow-Origin", "*");
        // request->send(response);
        AsyncClient *client = request->client();

        // Write HTTP header
        String header = "HTTP/1.1 200 OK\r\n";
        header += "Content-Type: ";
        header += _STREAM_CONTENT_TYPE;
        header += "\r\n\r\n";
        client->write(header.c_str(), header.length());

        // Spawn a FreeRTOS task to continuously push frames
        xTaskCreatePinnedToCore(
            [](void *param) {
                AsyncClient *client = (AsyncClient *)param;

                while (client->connected()) {
                    camera_fb_t *fb = esp_camera_fb_get();
                    if (!fb) {
                        vTaskDelay(10 / portTICK_PERIOD_MS);
                        continue;
                    }
                    if (client->space() > 64) {
                        client->write(_STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
                        char part[64];
                        size_t len = snprintf(part, 64, _STREAM_PART, fb->len);
                        client->write(part, len);
                        size_t toWrite = fb->len;
                        size_t written = 0;

                        // Serial.printf("fb size: %d - client buf space: %d\n", fb->len, client->space());
                        while (toWrite > 0 && client->connected()) {
                            size_t space = client->space();
                            size_t chunk_size = toWrite > space ? space : toWrite;
                            size_t sent = client->write((const char *)fb->buf + written, chunk_size);
                            toWrite -= sent;
                            while (client->space() <= 2 && client->connected()) {
                                // Serial.printf("waiting for client space: %d\n", client->space());
                                vTaskDelay(50 / portTICK_PERIOD_MS);
                            }
                        }
                        client->write("\r\n", 2);
                        // } else {
                        // Serial.printf("client buffer has no room: \n client
                        // buffer: %d\n frame buffer: %dn", client->space(),
                        // fb->len);
                        // }
    
                        esp_camera_fb_return(fb);
                        // Yield to watchdog and WiFi
                        vTaskDelay(100 / portTICK_PERIOD_MS);  // ~20 fps cap
                    }
                }

                client->close();
                vTaskDelete(NULL);
            },
            "streamTask", 16384, client, 1, NULL, 1);  // larger stack
    });
    server.begin();
}

void loadSettings() {
    prefs.begin("catfeeder", true);
    time_t t = (time_t)prefs.getLong64("lastFeed", 0);
    if (t != 0) {
        localtime_r(&t, &lastFeedTime);
    }
    lastFeedAmount = prefs.getLong("lastFeedAmount", 0);
    size_t len = prefs.getBytes("schedule", &schedule,
                                SCHEDULE_MAX_SIZE * sizeof(ScheduleItem));
    scheduleSize = len / sizeof(ScheduleItem);
    for (int i = 0; i < scheduleSize; i++) {
        Serial.printf("Scheduled feeding: %d:%d amount: %d\n", schedule[i].hour,
                      schedule[i].minute, schedule[i].amount);
    }
    openBeaconThresholdRSSI =
        prefs.getInt("openBeaconRSSI", openBeaconThresholdRSSI);
    closeBeaconThresholdRSSI =
        prefs.getInt("closeBeaconRSSI", closeBeaconThresholdRSSI);

    Serial.printf("Configured threasholds - open %d - close: %d \n",
                  openBeaconThresholdRSSI, closeBeaconThresholdRSSI);

    size_t len2 =
        prefs.getBytes("cats", &cats, CATS_MAX_SIZE * sizeof(CatItem));
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
    config.jpeg_quality = 13;
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

    // Init servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    lidServo.setPeriodHertz(50);  // standard 50 Hz servo

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

    // Init stepper
    stepper.setMaxSpeed(400);
    stepper.setAcceleration(800);

    // Init time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Setup WIFI
    Serial.print("Init WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    for (int i = 0; i < CATS_MAX_SIZE; i++) {
        lastAvgRSSI[i] = -100;
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

void periodic() {
    unsigned long now = millis();
    if (now - lastLog > printPeriodicLog) {
        wl_status_t wifiStatus = WiFi.status();
        isConnected = wifiStatus == WL_CONNECTED;
        if (isConnected) {
            if (!isServerInitialized) {
                afterWifiConnected();
                Serial.printf("Address: %s\n", WiFi.localIP().toString());
            }
            if (!timeIsSet && getLocalTime(&timeinfo)) {
                Serial.println("Acquired NTP time");
                getLocalTime(&lastStartTime);
                timeIsSet = true;
            }
        } else {
            Serial.printf("Wifi status: %s\n", wifiStatusName[wifiStatus]);
        }
        lastLog = now;
    }
}

void checkLid(unsigned long now) {
    int targetBiggerRSSI = -100;
    int otherCatBiggerRSSI = -100;
    for (size_t i = 0; i < catsSize; i++) {
        int avg = lastAvgRSSI[i];
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
        bool targetCatIsClose = targetBiggerRSSI > openBeaconThresholdRSSI;
        bool targetCatIsNotClose =
            !targetCatIsClose && targetBiggerRSSI < closeBeaconThresholdRSSI;
        bool anotherCatIsCloserThanTarget =
            otherCatBiggerRSSI > openBeaconThresholdRSSI &&
            otherCatBiggerRSSI > targetBiggerRSSI;

        if ((targetCatIsNotClose || anotherCatIsCloserThanTarget) &&
            ((now - lastOpen) >= 3000) && lidOpen) {
            closeLid();
        }

        if (targetCatIsClose && !lidOpen) {
            openLid();
        }
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
                delay(2000);
            }
        }
    }
}

void updateStepper() {
    stepper.run();
    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
    }
}

void loop() {
    unsigned long now = millis();

    // Update servo
    periodic();
    updateSmoothMove();

    // Update stepper
    updateStepper();

    // Lid should be open or closed?
    checkLid(now);

    // Check schedule
    runSchedule();

    ArduinoOTA.handle();
}