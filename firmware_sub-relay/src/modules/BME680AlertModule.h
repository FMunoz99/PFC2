#pragma once
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && !defined(ARCH_NRF52) == 0

#include "MeshService.h"
#include "Router.h"
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include "../mesh/generated/meshtastic/telemetry.pb.h"

#define BME680_TELEMETRY_INTERVAL   10000
#define BME680_ALERT_INTERVAL        2000
#define BME680_TEMP_MAX       35.0f
#define BME680_HUMIDITY_MAX   85.0f
#define BME680_IAQ_MAX       200.0f
#define BME680_NODE_ID  "RAK1"

class BME680AlertModule : public SinglePortModule, private concurrency::OSThread
{
  private:
    uint32_t lastTelemetry = 0;
    uint32_t lastAlert     = 0;

    bool isCritical(float temp, float humidity, float iaq);
    void sendBME680Message(float temp, float humidity, float pressure, float iaq, bool critical);

  public:
    BME680AlertModule();
    int32_t runOnce() override;
    bool wantUIFrame() override { return false; }
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
};

extern BME680AlertModule *bme680AlertModule;

#endif