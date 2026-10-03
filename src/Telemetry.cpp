#include "Telemetry.h"
#include "TelemetryPage.h"
#include <WiFi.h>

void Telemetry::begin(const char* ssid, const char* password) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);

    _ws.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type,
                       void* arg, uint8_t* data, size_t len) {
        if (type != WS_EVT_DATA) return;
        auto* info = static_cast<AwsFrameInfo*>(arg);
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            handleMessage(data, len);
        }
    });
    _server.addHandler(&_ws);
    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html", INDEX_HTML);
    });
    _server.begin();

    _queue = xQueueCreate(64, sizeof(Sample));
    // Core 0 (WiFi core), low priority: must never compete with the control loop on core 1
    xTaskCreatePinnedToCore(taskEntry, "telemetry", 4096, this, 1, nullptr, 0);
}

void Telemetry::send(float angle, float setpoint, float output) {
    const uint32_t now = millis();
    if (now - _lastSampleMs < SAMPLE_INTERVAL_MS) return;
    _lastSampleMs = now;

    Sample s{now, angle, setpoint, output};
    xQueueSend(_queue, &s, 0);  // timeout 0 -> never blocks the control loop
}

void Telemetry::taskEntry(void* self) {
    static_cast<Telemetry*>(self)->taskLoop();
}

void Telemetry::taskLoop() {
    static char buf[512];
    size_t pos = 0;
    uint32_t lastFlush = millis();
    uint32_t lastClean = lastFlush;
    Sample s;

    for (;;) {
        if (xQueueReceive(_queue, &s, pdMS_TO_TICKS(FLUSH_INTERVAL_MS)) == pdTRUE) {
            if (sizeof(buf) - pos > 48) {  // room for one more line
                pos += snprintf(buf + pos, sizeof(buf) - pos, "%lu,%.2f,%.2f,%.1f\n",
                                static_cast<unsigned long>(s.t), s.angle, s.setpoint, s.output);
            }
        }

        const uint32_t now = millis();
        if (now - lastFlush >= FLUSH_INTERVAL_MS) {
            lastFlush = now;
            if (pos > 0 && _ws.count() > 0 && _ws.availableForWriteAll()) {
                _ws.textAll(buf, pos);   // one packet with several samples
            }
            pos = 0;
            if (now - lastClean > 1000) {
                lastClean = now;
                _ws.cleanupClients();
            }
        }
    }
}

void Telemetry::handleMessage(const uint8_t* data, size_t len) {
    char buf[32];
    len = min(len, sizeof(buf) - 1);
    memcpy(buf, data, len);
    buf[len] = '\0';

    char* eq = strchr(buf, '=');
    if (!eq || !_onParam) return;
    *eq = '\0';
    _onParam(String(buf), atof(eq + 1));
}
