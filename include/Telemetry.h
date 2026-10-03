#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <functional>

// WiFi telemetry. The control loop only pushes samples into a queue (never blocks);
// a separate low-priority task on core 0 batches them and talks to the network.
class Telemetry {
public:
    using ParamCallback = std::function<void(const String& name, float value)>;

    void begin(const char* ssid = "Balancer", const char* password = "balance123");
    void setParamCallback(ParamCallback cb) { _onParam = cb; }

    // Call every control cycle. Non-blocking: drops the sample if the queue is full.
    void send(float angle, float setpoint, float output);

private:
    struct Sample { uint32_t t; float angle, setpoint, output; };

    static void taskEntry(void* self);
    void taskLoop();
    void handleMessage(const uint8_t* data, size_t len);

    static constexpr uint32_t SAMPLE_INTERVAL_MS = 50;   // 20 Hz sampling
    static constexpr uint32_t FLUSH_INTERVAL_MS  = 250;  // one WebSocket message per 250 ms

    AsyncWebServer _server{80};
    AsyncWebSocket _ws{"/ws"};
    ParamCallback  _onParam;
    QueueHandle_t  _queue = nullptr;
    uint32_t       _lastSampleMs = 0;
};