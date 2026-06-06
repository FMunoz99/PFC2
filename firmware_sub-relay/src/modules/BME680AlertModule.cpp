#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR && !defined(ARCH_NRF52) == 0

#include "BME680AlertModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"

BME680AlertModule *bme680AlertModule;

BME680AlertModule::BME680AlertModule()
    : SinglePortModule("BME680Alert", meshtastic_PortNum_TELEMETRY_APP),
      OSThread("BME680AlertModule")
{
}

bool BME680AlertModule::isCritical(float temp, float humidity, float iaq)
{
    return (temp > BME680_TEMP_MAX) ||
           (humidity > BME680_HUMIDITY_MAX) ||
           (iaq > BME680_IAQ_MAX);
}

ProcessMessage BME680AlertModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Solo procesar paquetes de telemetría propios
    if (mp.from != nodeDB->getNodeNum())
        return ProcessMessage::CONTINUE;

    meshtastic_Telemetry t = meshtastic_Telemetry_init_zero;
    if (!pb_decode_from_bytes(mp.decoded.payload.bytes, mp.decoded.payload.size, &meshtastic_Telemetry_msg, &t))
        return ProcessMessage::CONTINUE;

    if (t.which_variant != meshtastic_Telemetry_environment_metrics_tag)
        return ProcessMessage::CONTINUE;

    float temp     = t.variant.environment_metrics.temperature;
    float humidity = t.variant.environment_metrics.relative_humidity;
    float iaq      = (float)t.variant.environment_metrics.iaq;
    float pressure = t.variant.environment_metrics.barometric_pressure;

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

    return ProcessMessage::CONTINUE;
}

int32_t BME680AlertModule::runOnce()
{
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