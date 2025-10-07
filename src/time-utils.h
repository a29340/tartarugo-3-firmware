#include <Arduino.h>

struct tm timeinfo;
boolean timeIsSet = false;
struct tm lastFeedTime;
struct tm lastStartTime;

struct ScheduleItem
{
    uint8_t hour;
    uint8_t minute;
    uint16_t amount;
};

String printDateTime(const tm* dateTime)
{
    // 2015-03-25T12:00:00Z
    char buffer[26];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%X", dateTime);
    return String(buffer);
}
