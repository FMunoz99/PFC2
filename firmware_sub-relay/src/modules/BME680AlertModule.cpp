#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && (__has_include(<bsec2.h>) || __has_include(<Adafruit_BME680.h>))

#include "BME680AlertModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"

#if __has_include(<bsec2.h>)
#include <Wire.h>
#include <bsec2.h>
static Bsec2 alertBme;
static bool alertBmeReady = false;
#endif

BME680AlertModule *bme680AlertModule;

BME680AlertModule::BME680AlertModule()
    : ProtobufModule("BME680Alert", meshtastic_PortNum_TELEMETRY_APP, &meshtastic_Telemetry_msg),
      OSThread("BME680AlertModule")
{
}

bool BME680AlertModule::isCritical(float temp, float humidity, float iaq)
{
    return (temp > BME680_TEMP_MAX) ||
           (humidity > BME680_HUMIDITY_MAX) ||
           (iaq > BME680_IAQ_MAX);
}

bool BME680AlertModule::handleReceivedProtobuf(const meshtastic_MeshPacket &mp, meshtastic_Telemetry *t)
{
    return false;
}

int32_t BME680AlertModule::runOnce()
{
#if __has_include(<bsec2.h>)
    if (!alertBmeReady) {
        if (alertBme.begin(0x76, Wire)) {
            bsecSensor sensorList[] = {
                BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
                BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
                BSEC_OUTPUT_RAW_PRESSURE,
                BSEC_OUTPUT_IAQ
            };
            alertBme.updateSubscription(sensorList, 4, BSEC_SAMPLE_RATE_LP);
            alertBmeReady = true;
            LOG_INFO("BME680Alert: sensor listo\n");
        } else {
            LOG_WARN("BME680Alert: sensor no encontrado\n");
            return 10000;
        }
    }

    if (!alertBme.run())
        return 1000;

    float temp     = alertBme.getData(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE).signal;
    float humidity = alertBme.getData(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY).signal;
    float pressure = alertBme.getData(BSEC_OUTPUT_RAW_PRESSURE).signal / 100.0f;
    float iaq      = alertBme.getData(BSEC_OUTPUT_IAQ).signal;

    if (temp == 0.0f && humidity == 0.0f)
        return 1000;

    uint32_t now = millis();
    bool critical = isCritical(temp, humidity, iaq);

    if (critical) {
        if (now - lastAlert >= BME680_ALERT_INTERVAL) {
            LOG_WARN("BME680Alert: CRITICO T=%.1f H=%.1f IAQ=%.0f\n", temp, humidity, iaq);
            sendBME680Message(temp, humidity, pressure, iaq, true);
            lastAlert = now;
        }
    } else {
        if (now - lastTelemetry >= BME680_TELEMETRY_INTERVAL) {
            LOG_INFO("BME680Alert: T=%.1f H=%.1f P=%.1f IAQ=%.0f\n", temp, humidity, pressure, iaq);
            sendBME680Message(temp, humidity, pressure, iaq, false);
            lastTelemetry = now;
        }
    }
#endif
    return 1000;
}

void BME680AlertModule::sendBME680Message(float temp, float humidity, float pressure, float iaq, bool critical)
{
    char mensaje[100];

    if (critical) {
        snprintf(mensaje, sizeof(mensaje),
                 "[%s] ALERTA AMB: T=%.1fC H=%.1f%% IAQ=%.0f - CRITICO",
                 BME680_NODE_ID, temp, humidity, iaq);
    } else {
        snprintf(mensaje, sizeof(mensaje),
                 "[%s] AMB: T=%.1fC H=%.1f%% P=%.1fhPa IAQ=%.0f",
                 BME680_NODE_ID, temp, humidity, pressure, iaq);
    }

    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->priority = critical ? meshtastic_MeshPacket_Priority_HIGH
                           : meshtastic_MeshPacket_Priority_DEFAULT;

    memcpy(p->decoded.payload.bytes, mensaje, strlen(mensaje));
    p->decoded.payload.size = strlen(mensaje);
    p->to      = NODENUM_BROADCAST;
    p->channel = 0;

    LOG_INFO("BME680Alert: Enviando: %s\n", mensaje);
    p->want_ack = false;
    service->sendToMesh(p);
}

#endif