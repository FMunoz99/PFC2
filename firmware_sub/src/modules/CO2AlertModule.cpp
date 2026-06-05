#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "CO2AlertModule.h"
#include "MeshService.h"
#include "NodeDB.h"
#include "Router.h"

#ifdef HAS_SCREEN
#include "graphics/Screen.h"
#include <OLEDDisplay.h>
#include <OLEDDisplayUi.h>
#endif

CO2AlertModule *co2AlertModule;

CO2AlertModule::CO2AlertModule()
    : SinglePortModule("CO2Alert", meshtastic_PortNum_PRIVATE_APP),
      OSThread("CO2AlertModule")
{
}

int32_t CO2AlertModule::runOnce()
{
    static bool initialized = false;
    if (!initialized) {
        sensor.runOnce();
        initialized = true;
        return 5000; // esperar 5s antes de primera lectura
    }

    int ppm = sensor.getCO2PPM();
    uint32_t now = millis();

    // Lectura invalida, ignorar
    if (ppm < 100 || ppm > 10000) {
        LOG_DEBUG("CO2Alert: Esperando sensor (%d ppm raw)\n", ppm);
        return 1000; // ← antes era 5000, reducir para no bloquear
    }

    if (sensor.isCritical()) {
        // Alerta critica: enviar cada 5 segundos
        if (now - lastAlert >= CO2_ALERT_INTERVAL) {
            LOG_WARN("CO2Alert: CRITICO %d ppm - Enviando alerta\n", ppm);
            sendCO2Message(ppm, true);
            lastAlert = now;
        }
    } else {
        // Telemetria normal: enviar cada 30 segundos
        if (now - lastTelemetry >= CO2_TELEMETRY_INTERVAL) {
            LOG_INFO("CO2Alert: %d ppm - Enviando telemetria\n", ppm);
            sendCO2Message(ppm, false);
            lastTelemetry = now;
        }
    }

    return 1000; // revisar cada 1 segundo
}

void CO2AlertModule::sendCO2Message(int ppm, bool critical)
{
    char mensaje[80];

    if (critical) {
        snprintf(mensaje, sizeof(mensaje),
                "[%s] ALERTA CO2: %d ppm - NIVEL CRITICO", CO2_NODE_ID, ppm);
    } else if (sensor.isWarning()) {
        snprintf(mensaje, sizeof(mensaje),
                "[%s] AVISO CO2: %d ppm - Nivel elevado", CO2_NODE_ID, ppm);
    } else {
        snprintf(mensaje, sizeof(mensaje),
                "[%s] CO2: %d ppm - Normal", CO2_NODE_ID, ppm);
    }

    // Crear paquete mesh
    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;

    if (critical) {
        p->priority = meshtastic_MeshPacket_Priority_HIGH;
    } else if (sensor.isWarning()) {
        p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    } else {
        p->priority = meshtastic_MeshPacket_Priority_DEFAULT;
    }

    memcpy(p->decoded.payload.bytes, mensaje, strlen(mensaje));
    p->decoded.payload.size = strlen(mensaje);
    p->to = NODENUM_BROADCAST;
    p->channel = 0; // canal primario

    LOG_INFO("CO2Alert: Enviando: %s\n", mensaje);
    p->want_ack = false;
    service->sendToMesh(p);
}

void CO2AlertModule::drawFrame(OLEDDisplay *display, OLEDDisplayUiState *state,
                                int16_t x, int16_t y)
{
    int ppm = sensor.getLastPPM();

    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(ArialMT_Plain_10);
    display->drawString(x + 0,  y + 0,  "CO2 Monitor");
    display->drawHorizontalLine(x, y + 11, x + 128);

    if (ppm >= 100 && ppm <= 10000) {
        display->setFont(ArialMT_Plain_16);
        display->drawString(x + 0, y + 14, String(ppm) + " ppm");
        display->setFont(ArialMT_Plain_10);
        if (ppm >= 2000)
            display->drawString(x + 0, y + 34, "NIVEL CRITICO");
        else if (ppm >= 1000)
            display->drawString(x + 0, y + 34, "ADVERTENCIA");
        else
            display->drawString(x + 0, y + 34, "NORMAL");
    } else {
        display->setFont(ArialMT_Plain_10);
        display->drawString(x + 0, y + 20, "Sin sensor");
    }
}

#endif