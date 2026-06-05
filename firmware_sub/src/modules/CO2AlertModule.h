#pragma once
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "MeshService.h"
#include "Router.h"
#include "SinglePortModule.h"
#include "Telemetry/Sensor/MHZ19ESensor.h"
#include "concurrency/OSThread.h"

#define CO2_TELEMETRY_INTERVAL  2000
#define CO2_ALERT_INTERVAL       500

class CO2AlertModule : public SinglePortModule, private concurrency::OSThread
{
  private:
    void sendCO2Message(int ppm, bool isCritical);
    uint32_t lastTelemetry = 0;
    uint32_t lastAlert = 0;

  public:
    MHZ19ESensor sensor;
    CO2AlertModule();
    int32_t runOnce() override;

    // Frame para la pantalla OLED
    bool wantUIFrame() override { return true; }
    void drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                   int16_t x, int16_t y) override;
};

extern CO2AlertModule *co2AlertModule;

#endif