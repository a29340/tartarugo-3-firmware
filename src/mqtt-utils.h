// MQTT integration for Home Assistant.
//
// Every device gets its own topic namespace derived from its MAC address, so
// multiple feeders can share one broker (and one Home Assistant):
//   <base>/<mac>/state               JSON state, published periodically and after changes
//   <base>/<mac>/status              retained availability, "online" / "offline" (LWT)
//   <base>/<mac>/cmd/lid1            "auto" | "open" | "close"
//   <base>/<mac>/cmd/lid2            "open" | "close"
//   <base>/<mac>/cmd/feed            integer amount -> stored as feed amount (no feed yet)
//   <base>/<mac>/cmd/feed_now        button press -> feed with the stored amount
//   <base>/<mac>/cmd/threshold_open  integer RSSI open threshold
//   <base>/<mac>/cmd/threshold_close integer RSSI close threshold
// (<mac> is the device MAC without colons)
//
// Home Assistant entities are created automatically through MQTT discovery
// (homeassistant/<type>/<id>/config, published retained on every connect).
// Entity unique_ids embed the MAC, so each feeder shows up as its own device.
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
#define MQTT_DISCOVERY_PREFIX "homeassistant"
#define MQTT_TOPIC_LEN 48
#define MQTT_ENTITY_ID_LEN 48

constexpr unsigned long MQTT_RECONNECT_INTERVAL_MS = 15000;
constexpr unsigned long MQTT_STATE_PERIOD_MS = 30000;

WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

String mqttClientId;
char mqttMacNoColon[13];
char mqttStateTopic[MQTT_TOPIC_LEN];
char mqttStatusTopic[MQTT_TOPIC_LEN];
char mqttCmdLid1Topic[MQTT_TOPIC_LEN];
char mqttCmdLid2Topic[MQTT_TOPIC_LEN];
char mqttCmdFeedTopic[MQTT_TOPIC_LEN];
char mqttCmdFeedNowTopic[MQTT_TOPIC_LEN];
char mqttCmdThresholdOpenTopic[MQTT_TOPIC_LEN];
char mqttCmdThresholdCloseTopic[MQTT_TOPIC_LEN];
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
    device["identifiers"] = mqttClientId;
    device["name"] = "Tartarugo Cat Feeder";
    device["model"] = "ESP32 cat feeder";
    device["manufacturer"] = "Tartarugo";

    auto addCommon = [&](JsonDocument& doc)
    {
        doc["availability_topic"] = mqttStatusTopic;
        doc["payload_available"] = "online";
        doc["payload_not_available"] = "offline";
        doc["device"] = device;
    };

    char uid[MQTT_ENTITY_ID_LEN];
    JsonDocument d;

    snprintf(uid, sizeof(uid), "tartarugo_%s_lid1", mqttMacNoColon);
    d["name"] = "Lid 1";
    d["unique_id"] = uid;
    d["command_topic"] = mqttCmdLid1Topic;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.lid1 }}";
    JsonArray lid1Options = d["options"].to<JsonArray>();
    lid1Options.add("auto");
    lid1Options.add("open");
    lid1Options.add("close");
    addCommon(d);
    mqttSendConfig("select", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_lid2", mqttMacNoColon);
    d["name"] = "Lid 2";
    d["unique_id"] = uid;
    d["command_topic"] = mqttCmdLid2Topic;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.lid2 }}";
    JsonArray lid2Options = d["options"].to<JsonArray>();
    lid2Options.add("open");
    lid2Options.add("close");
    addCommon(d);
    mqttSendConfig("select", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_feed", mqttMacNoColon);
    d["name"] = "Feed amount";
    d["unique_id"] = uid;
    d["command_topic"] = mqttCmdFeedTopic;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.feedAmount }}";
    d["min"] = 1;
    d["max"] = 10000;
    d["step"] = 1;
    addCommon(d);
    mqttSendConfig("number", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_feed_button", mqttMacNoColon);
    d["name"] = "Feed";
    d["unique_id"] = uid;
    d["command_topic"] = mqttCmdFeedNowTopic;
    addCommon(d);
    mqttSendConfig("button", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_open_threshold", mqttMacNoColon);
    d["name"] = "Open threshold RSSI";
    d["unique_id"] = uid;
    d["command_topic"] = mqttCmdThresholdOpenTopic;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.openThreshold }}";
    d["min"] = -100;
    d["max"] = 0;
    d["step"] = 1;
    addCommon(d);
    mqttSendConfig("number", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_close_threshold", mqttMacNoColon);
    d["name"] = "Close threshold RSSI";
    d["unique_id"] = uid;
    d["command_topic"] = mqttCmdThresholdCloseTopic;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.closeThreshold }}";
    d["min"] = -100;
    d["max"] = 0;
    d["step"] = 1;
    addCommon(d);
    mqttSendConfig("number", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_wifi_rssi", mqttMacNoColon);
    d["name"] = "WiFi RSSI";
    d["unique_id"] = uid;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.wifiRssi }}";
    d["unit_of_measurement"] = "dBm";
    d["device_class"] = "signal_strength";
    addCommon(d);
    mqttSendConfig("sensor", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_last_feed", mqttMacNoColon);
    d["name"] = "Last feed";
    d["unique_id"] = uid;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.lastFeedTime }}";
    d["device_class"] = "timestamp";
    addCommon(d);
    mqttSendConfig("sensor", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_last_feed_amount", mqttMacNoColon);
    d["name"] = "Last feed amount";
    d["unique_id"] = uid;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json.lastFeedAmount }}";
    addCommon(d);
    mqttSendConfig("sensor", uid, d);

    d.clear();
    snprintf(uid, sizeof(uid), "tartarugo_%s_status", mqttMacNoColon);
    d["name"] = "Status";
    d["unique_id"] = uid;
    d["state_topic"] = mqttStateTopic;
    d["value_template"] = "{{ value_json | tojson }}";
    d["json_attributes_template"] = "{{ value_json }}";
    addCommon(d);
    mqttSendConfig("sensor", uid, d);
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
    if (!mqttClient.publish(mqttStateTopic, out.c_str()))
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

    if (strcmp(topic, mqttCmdLid1Topic) == 0)
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
    else if (strcmp(topic, mqttCmdLid2Topic) == 0)
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
    else if (strcmp(topic, mqttCmdFeedTopic) == 0)
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
    else if (strcmp(topic, mqttCmdFeedNowTopic) == 0)
    {
        if (cmd != "PRESS")
        {
            Serial.printf("MQTT: unknown feed_now payload '%s'\n", cmd.c_str());
            return;
        }
        feedAmount(feedAmountSetting);
        saveLastFeed();
    }
    else if (strcmp(topic, mqttCmdThresholdOpenTopic) == 0)
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
    else if (strcmp(topic, mqttCmdThresholdCloseTopic) == 0)
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
    int j = 0;
    for (int i = 0; i < mac.length() && j < 12; i++)
    {
        if (mac[i] != ':')
        {
            mqttMacNoColon[j++] = mac[i];
        }
    }
    mqttMacNoColon[j] = '\0';

    mqttClientId = String(MQTT_BASE_TOPIC) + "-" + mac;
    snprintf(mqttStateTopic, sizeof(mqttStateTopic),
             MQTT_BASE_TOPIC "/%s/state", mqttMacNoColon);
    snprintf(mqttStatusTopic, sizeof(mqttStatusTopic),
             MQTT_BASE_TOPIC "/%s/status", mqttMacNoColon);
    snprintf(mqttCmdLid1Topic, sizeof(mqttCmdLid1Topic),
             MQTT_BASE_TOPIC "/%s/cmd/lid1", mqttMacNoColon);
    snprintf(mqttCmdLid2Topic, sizeof(mqttCmdLid2Topic),
             MQTT_BASE_TOPIC "/%s/cmd/lid2", mqttMacNoColon);
    snprintf(mqttCmdFeedTopic, sizeof(mqttCmdFeedTopic),
             MQTT_BASE_TOPIC "/%s/cmd/feed", mqttMacNoColon);
    snprintf(mqttCmdFeedNowTopic, sizeof(mqttCmdFeedNowTopic),
             MQTT_BASE_TOPIC "/%s/cmd/feed_now", mqttMacNoColon);
    snprintf(mqttCmdThresholdOpenTopic, sizeof(mqttCmdThresholdOpenTopic),
             MQTT_BASE_TOPIC "/%s/cmd/threshold_open", mqttMacNoColon);
    snprintf(mqttCmdThresholdCloseTopic, sizeof(mqttCmdThresholdCloseTopic),
             MQTT_BASE_TOPIC "/%s/cmd/threshold_close", mqttMacNoColon);

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
        mqttStatusTopic,
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

    mqttClient.publish(mqttStatusTopic, "online", true);
    mqttClient.subscribe(mqttCmdLid1Topic);
    mqttClient.subscribe(mqttCmdLid2Topic);
    mqttClient.subscribe(mqttCmdFeedTopic);
    mqttClient.subscribe(mqttCmdFeedNowTopic);
    mqttClient.subscribe(mqttCmdThresholdOpenTopic);
    mqttClient.subscribe(mqttCmdThresholdCloseTopic);
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
