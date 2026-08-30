// MQTT integration for Home Assistant.
//
// Topics (all under MQTT_BASE_TOPIC):
//   state               JSON state, published periodically and after changes
//   status              retained availability, "online" / "offline" (LWT)
//   cmd/lid1            "auto" | "open" | "close"
//   cmd/lid2            "open" | "close"
//   cmd/feed            integer amount -> stored as feed amount (no feed yet)
//   cmd/feed_now        button press -> feed with the stored amount
//   cmd/threshold_open  integer RSSI open threshold
//   cmd/threshold_close integer RSSI close threshold
//
// Home Assistant entities are created automatically through MQTT discovery
// (homeassistant/<type>/<id>/config, published retained on every connect).
//
// This file is included at the end of main.cpp, so it may use the globals
// and helpers defined there (cats, thresholds, saveSettings, ...).

#include <Arduino.h>
#include <ArduinoJson.h>

// PubSubClient's packet size must be >= MQTT_MAX_PACKET_SIZE (set to 1024 in
// platformio.ini build_flags) in EVERY translation unit, otherwise publish()
// silently rejects payloads over the default 256 bytes (discovery configs).
#include <PubSubClient.h>
#include "MQTTCredentials.h"

#define MQTT_BASE_TOPIC "tartarugo"
#define MQTT_STATE_TOPIC MQTT_BASE_TOPIC "/state"
#define MQTT_STATUS_TOPIC MQTT_BASE_TOPIC "/status"
#define MQTT_CMD_LID1 MQTT_BASE_TOPIC "/cmd/lid1"
#define MQTT_CMD_LID2 MQTT_BASE_TOPIC "/cmd/lid2"
#define MQTT_CMD_FEED MQTT_BASE_TOPIC "/cmd/feed"
#define MQTT_CMD_FEED_NOW MQTT_BASE_TOPIC "/cmd/feed_now"
#define MQTT_CMD_THRESHOLD_OPEN MQTT_BASE_TOPIC "/cmd/threshold_open"
#define MQTT_CMD_THRESHOLD_CLOSE MQTT_BASE_TOPIC "/cmd/threshold_close"
#define MQTT_DISCOVERY_PREFIX "homeassistant"

constexpr unsigned long MQTT_RECONNECT_INTERVAL_MS = 15000;
constexpr unsigned long MQTT_STATE_PERIOD_MS = 30000;

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

String mqttClientId;
unsigned long lastMqttAttempt = 0;
unsigned long lastMqttStatePublish = 0;
char mqttCommandBuffer[64];

void mqttSendConfig(const char* type, const char* entityId, JsonDocument& doc)
{
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/%s/%s/config",
             MQTT_DISCOVERY_PREFIX, type, entityId);
    String out;
    serializeJson(doc, out);
    if (!mqttClient.publish(topic, out.c_str(), true))
    {
        Serial.printf("MQTT: discovery publish failed for %s/%s (len=%u)\n",
                      type, entityId, out.length());
    }
}

void mqttPublishDiscovery()
{
    if (!mqttClient.connected()) return;

    JsonDocument device;
    device["identifiers"] = "tartarugo";
    device["name"] = "Tartarugo Cat Feeder";
    device["model"] = "ESP32 cat feeder";
    device["manufacturer"] = "Tartarugo";

    auto addCommon = [&](JsonDocument& doc)
    {
        doc["availability_topic"] = MQTT_STATUS_TOPIC;
        doc["payload_available"] = "online";
        doc["payload_not_available"] = "offline";
        doc["device"] = device;
    };

    JsonDocument d;

    d["name"] = "Lid 1";
    d["unique_id"] = "tartarugo_lid1";
    d["command_topic"] = MQTT_CMD_LID1;
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.lid1 }}";
    JsonArray lid1Options = d["options"].to<JsonArray>();
    lid1Options.add("auto");
    lid1Options.add("open");
    lid1Options.add("close");
    addCommon(d);
    mqttSendConfig("select", "tartarugo_lid1", d);

    d.clear();
    d["name"] = "Lid 2";
    d["unique_id"] = "tartarugo_lid2";
    d["command_topic"] = MQTT_CMD_LID2;
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.lid2 }}";
    JsonArray lid2Options = d["options"].to<JsonArray>();
    lid2Options.add("open");
    lid2Options.add("close");
    addCommon(d);
    mqttSendConfig("select", "tartarugo_lid2", d);

    d.clear();
    d["name"] = "Feed amount";
    d["unique_id"] = "tartarugo_feed";
    d["command_topic"] = MQTT_CMD_FEED;
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.feedAmount }}";
    d["min"] = 1;
    d["max"] = 10000;
    d["step"] = 1;
    addCommon(d);
    mqttSendConfig("number", "tartarugo_feed", d);

    d.clear();
    d["name"] = "Feed";
    d["unique_id"] = "tartarugo_feed_button";
    d["command_topic"] = MQTT_CMD_FEED_NOW;
    addCommon(d);
    mqttSendConfig("button", "tartarugo_feed_button", d);

    d.clear();
    d["name"] = "Open threshold RSSI";
    d["unique_id"] = "tartarugo_open_threshold";
    d["command_topic"] = MQTT_CMD_THRESHOLD_OPEN;
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.openThreshold }}";
    d["min"] = -100;
    d["max"] = 0;
    d["step"] = 1;
    addCommon(d);
    mqttSendConfig("number", "tartarugo_open_threshold", d);

    d.clear();
    d["name"] = "Close threshold RSSI";
    d["unique_id"] = "tartarugo_close_threshold";
    d["command_topic"] = MQTT_CMD_THRESHOLD_CLOSE;
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.closeThreshold }}";
    d["min"] = -100;
    d["max"] = 0;
    d["step"] = 1;
    addCommon(d);
    mqttSendConfig("number", "tartarugo_close_threshold", d);

    d.clear();
    d["name"] = "WiFi RSSI";
    d["unique_id"] = "tartarugo_wifi_rssi";
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.wifiRssi }}";
    d["unit_of_measurement"] = "dBm";
    d["device_class"] = "signal_strength";
    addCommon(d);
    mqttSendConfig("sensor", "tartarugo_wifi_rssi", d);

    d.clear();
    d["name"] = "Last feed";
    d["unique_id"] = "tartarugo_last_feed";
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.lastFeedTime }}";
    d["device_class"] = "timestamp";
    addCommon(d);
    mqttSendConfig("sensor", "tartarugo_last_feed", d);

    d.clear();
    d["name"] = "Last feed amount";
    d["unique_id"] = "tartarugo_last_feed_amount";
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json.lastFeedAmount }}";
    addCommon(d);
    mqttSendConfig("sensor", "tartarugo_last_feed_amount", d);

    d.clear();
    d["name"] = "Status";
    d["unique_id"] = "tartarugo_status";
    d["state_topic"] = MQTT_STATE_TOPIC;
    d["value_template"] = "{{ value_json | tojson }}";
    d["json_attributes_template"] = "{{ value_json }}";
    addCommon(d);
    mqttSendConfig("sensor", "tartarugo_status", d);
}

void mqttPublishState()
{
    if (!mqttClient.connected()) return;

    JsonDocument doc;
    doc["lid1"] = lidOverride ? (lidOpen ? "open" : "close") : "auto";
    doc["lid2"] = lid2Open ? "open" : "close";
    doc["wifiRssi"] = WiFi.RSSI();
    doc["openThreshold"] = openBeaconThresholdRSSI;
    doc["closeThreshold"] = closeBeaconThresholdRSSI;
    doc["feedAmount"] = feedAmountSetting;
    doc["lastFeedAmount"] = lastFeedAmount;
    doc["lastFeedTime"] =
        lastFeedAmount == 0 ? "" : printDateTime(&lastFeedTime).c_str();
    const JsonArray catsJson = doc["cats"].to<JsonArray>();
    for (size_t i = 0; i < catsSize; i++)
    {
        JsonObject cat = catsJson.add<JsonObject>();
        cat["name"] = cats[i].name;
        cat["rssi"] = lastAvgRSSI[i];
        cat["canFeed"] = cats[i].canFeed;
        cat["lastSeen"] = lastSeenTimestamp[i];
    }
    String out;
    serializeJson(doc, out);
    if (!mqttClient.publish(MQTT_STATE_TOPIC, out.c_str()))
    {
        Serial.printf("MQTT: state publish failed (len=%u)\n", out.length());
    }
    lastMqttStatePublish = millis();
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
    if (length >= sizeof(mqttCommandBuffer))
    {
        length = sizeof(mqttCommandBuffer) - 1;
    }
    memcpy(mqttCommandBuffer, payload, length);
    mqttCommandBuffer[length] = '\0';
    const String cmd = mqttCommandBuffer;

    if (strcmp(topic, MQTT_CMD_LID1) == 0)
    {
        if (cmd == "auto")
        {
            lidOverride = false;
        }
        else if (cmd == "open")
        {
            lidOverride = true;
            openLid(LID_1);
        }
        else if (cmd == "close")
        {
            lidOverride = true;
            closeLid(LID_1);
        }
        else
        {
            Serial.printf("MQTT: unknown lid1 command '%s'\n", cmd.c_str());
            return;
        }
    }
    else if (strcmp(topic, MQTT_CMD_LID2) == 0)
    {
        if (cmd == "open")
        {
            lidOverride = true;
            openLid(LID_2);
        }
        else if (cmd == "close")
        {
            closeLid(LID_2);
        }
        else
        {
            Serial.printf("MQTT: unknown lid2 command '%s'\n", cmd.c_str());
            return;
        }
    }
    else if (strcmp(topic, MQTT_CMD_FEED) == 0)
    {
        const int amount = cmd.toInt();
        if (amount <= 0 || amount > 10000)
        {
            Serial.printf("MQTT: invalid feed amount '%s'\n", cmd.c_str());
            return;
        }
        feedAmountSetting = amount;
        saveSettings();
    }
    else if (strcmp(topic, MQTT_CMD_FEED_NOW) == 0)
    {
        if (cmd != "PRESS")
        {
            Serial.printf("MQTT: unknown feed_now payload '%s'\n", cmd.c_str());
            return;
        }
        feedAmount(feedAmountSetting);
        saveLastFeed();
    }
    else if (strcmp(topic, MQTT_CMD_THRESHOLD_OPEN) == 0)
    {
        const int value = cmd.toInt();
        if (value < -100 || value > 0)
        {
            Serial.printf("MQTT: invalid open threshold '%s'\n", cmd.c_str());
            return;
        }
        openBeaconThresholdRSSI = value;
        saveSettings();
    }
    else if (strcmp(topic, MQTT_CMD_THRESHOLD_CLOSE) == 0)
    {
        const int value = cmd.toInt();
        if (value < -100 || value > 0)
        {
            Serial.printf("MQTT: invalid close threshold '%s'\n", cmd.c_str());
            return;
        }
        closeBeaconThresholdRSSI = value;
        saveSettings();
    }
    else
    {
        return;
    }

    mqttPublishState();
}

void mqttInit()
{
    const String mac = WiFi.macAddress();
    mqttClientId = String("tartarugo-") + mac;
    mqttClient.setServer(mqttHost, mqttPort);
    mqttClient.setCallback(mqttCallback);
}

void mqttReconnectIfNeeded()
{
    if (mqttClient.connected()) return;
    if (WiFi.status() != WL_CONNECTED) return;
    const unsigned long now = millis();
    if (now - lastMqttAttempt < MQTT_RECONNECT_INTERVAL_MS) return;
    lastMqttAttempt = now;

    Serial.printf("MQTT: connecting to %s:%d\n", mqttHost, mqttPort);
    const bool connected = mqttClient.connect(
        mqttClientId.c_str(),
        mqttUsername,
        mqttPassword,
        MQTT_STATUS_TOPIC,
        0,
        true,
        "offline",
        true);
    if (!connected)
    {
        Serial.printf("MQTT: connection failed (rc=%d), retrying\n",
                      mqttClient.state());
        return;
    }
    Serial.println("MQTT: connected");

    mqttClient.publish(MQTT_STATUS_TOPIC, "online", true);
    mqttClient.subscribe(MQTT_CMD_LID1);
    mqttClient.subscribe(MQTT_CMD_LID2);
    mqttClient.subscribe(MQTT_CMD_FEED);
    mqttClient.subscribe(MQTT_CMD_FEED_NOW);
    mqttClient.subscribe(MQTT_CMD_THRESHOLD_OPEN);
    mqttClient.subscribe(MQTT_CMD_THRESHOLD_CLOSE);
    mqttPublishDiscovery();
    mqttPublishState();
}

void mqttUpdate()
{
    mqttReconnectIfNeeded();
    mqttClient.loop();
    if (millis() - lastMqttStatePublish >= MQTT_STATE_PERIOD_MS)
    {
        mqttPublishState();
    }
}
