#include "configuration.h"
#include "UARTBridgeModule.h"
#include "NodeDB.h"
#include "Router.h"
#include <Arduino.h>

// Definición de la variable global
UARTBridgeModule *uartBridgeModule = nullptr;

// ── Constructor ────────────────────────────────────────────────
UARTBridgeModule::UARTBridgeModule()
    : MeshModule("UARTBridge")
{
    // Interceptar TODOS los paquetes entrantes (no filtramos por portnum)
    //boundPort = meshtastic_PortNum_UNKNOWN_APP;  // catch-all

    initUART();
}

// ── Inicialización UART ────────────────────────────────────────
void UARTBridgeModule::initUART()
{
    // Serial2 en LilyGO T3_V1.6.1: GPIO12=TX, GPIO34=RX (input-only)
    // NOTA: Serial2 es UART2 del ESP32. No interfiere con USB (UART0)
    //       ni con el sensor CO2 (UART1) si existiera en otra placa.
    Serial2.begin(BRIDGE_UART_BAUD, SERIAL_8N1, BRIDGE_UART_RX_PIN, BRIDGE_UART_TX_PIN);

    pinMode(BRIDGE_LED_PIN, OUTPUT);
    digitalWrite(BRIDGE_LED_PIN, LOW);

    uartReady = true;
    LOG_INFO("UARTBridge: Serial2 listo TX=GPIO%d RX=GPIO%d @ %d baud\n",
             BRIDGE_UART_TX_PIN, BRIDGE_UART_RX_PIN, BRIDGE_UART_BAUD);
}

// ── Handler principal — llamado por Meshtastic al recibir paquete ─
ProcessMessage UARTBridgeModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Ignorar paquetes que originamos nosotros mismos
    if (mp.from == nodeDB->getNodeNum()) {
        return ProcessMessage::CONTINUE;
    }

    LOG_INFO("UARTBridge: Paquete recibido de 0x%08x, port=%d\n",
             mp.from, mp.decoded.portnum);

    if (!uartReady) {
        LOG_WARN("UARTBridge: UART no listo, descartando paquete\n");
        return ProcessMessage::CONTINUE;
    }

    bool ok = sendPacketUART(mp);
    if (ok) {
        blinkLED(1, 150);  // 1 parpadeo = enviado OK
    } else {
        blinkLED(3, 80);   // 3 parpadeos rápidos = payload muy largo, truncado
    }

    // CONTINUE para que otros módulos también procesen el paquete
    return ProcessMessage::CONTINUE;
}

// ── Construcción y envío del JSON ──────────────────────────────
bool UARTBridgeModule::sendPacketUART(const meshtastic_MeshPacket &mp)
{
    // ── Extraer campos disponibles ──────────────────────────────
    uint32_t from_id  = mp.from;
    uint32_t to_id    = mp.to;
    int32_t  rssi     = mp.rx_rssi;
    float    snr      = mp.rx_snr;
    uint8_t  hops     = (mp.hop_start > 0 && mp.hop_limit <= mp.hop_start)
                        ? (mp.hop_start - mp.hop_limit)
                        : 0;
    uint32_t ts       = mp.rx_time;   // Unix timestamp (segundos)
    String   portStr  = portNumToString((meshtastic_PortNum)mp.decoded.portnum);
    String   payload  = extractPayloadText(mp);

    // ── Determinar prioridad para SDN ──────────────────────────
    // El gateway SDN puede re-clasificar, pero enviamos la prioridad
    // del paquete original para facilitar la tarea.
    String priority = "NORMAL";
    if (mp.priority >= meshtastic_MeshPacket_Priority_HIGH) {
        priority = "HIGH";
    } else if (mp.priority >= meshtastic_MeshPacket_Priority_RELIABLE) {
        priority = "MEDIUM";
    }

    // ── Construcción JSON compacto ─────────────────────────────
    // Formato: {"f":"<hex>","t":"<hex>","p":"<port>","rs":<rssi>,
    //           "sn":<snr>,"h":<hops>,"ts":<ts>,"pr":"<prio>","d":"<payload>"}
    // Mantenemos claves cortas para minimizar bytes en LoRaWAN.
    char buf[BRIDGE_MAX_PAYLOAD + 64];  // margen para construcción
    int len = snprintf(buf, sizeof(buf),
        "{\"f\":\"%08lx\",\"t\":\"%08lx\",\"p\":\"%s\","
        "\"rs\":%ld,\"sn\":%.1f,\"h\":%u,"
        "\"ts\":%lu,\"pr\":\"%s\",\"d\":\"%s\"}\n",
        (unsigned long)from_id,
        (unsigned long)to_id,
        portStr.c_str(),
        (long)rssi,
        (double)snr,
        (unsigned)hops,
        (unsigned long)ts,
        priority.c_str(),
        payload.c_str()
    );

    bool truncated = false;
    if (len >= BRIDGE_MAX_PAYLOAD) {
        // Truncar payload para no superar límite LoRaWAN
        // Reconstruimos sin el campo "d" si no cabe
        len = snprintf(buf, sizeof(buf),
            "{\"f\":\"%08lx\",\"t\":\"%08lx\",\"p\":\"%s\","
            "\"rs\":%ld,\"sn\":%.1f,\"h\":%u,"
            "\"ts\":%lu,\"pr\":\"%s\",\"d\":\"TRUNC\"}\n",
            (unsigned long)from_id,
            (unsigned long)to_id,
            portStr.c_str(),
            (long)rssi,
            (double)snr,
            (unsigned)hops,
            (unsigned long)ts,
            priority.c_str()
        );
        truncated = true;
        LOG_WARN("UARTBridge: Payload truncado — original demasiado largo\n");
    }

    Serial2.print(buf);
    Serial2.flush();

    LOG_INFO("UARTBridge: Enviado por UART (%d bytes)%s: %s",
             len, truncated ? " [TRUNCADO]" : "", buf);

    return !truncated;
}

// ── Helpers ────────────────────────────────────────────────────
String UARTBridgeModule::portNumToString(meshtastic_PortNum port)
{
    switch (port) {
        case meshtastic_PortNum_TEXT_MESSAGE_APP:    return "TEXT";
        case meshtastic_PortNum_TELEMETRY_APP:       return "TELEM";
        case meshtastic_PortNum_POSITION_APP:        return "POS";
        case meshtastic_PortNum_NODEINFO_APP:        return "NINFO";
        case meshtastic_PortNum_ROUTING_APP:         return "ROUTE";
        case meshtastic_PortNum_ADMIN_APP:           return "ADMIN";
        case meshtastic_PortNum_RANGE_TEST_APP:      return "RANGE";
        case meshtastic_PortNum_DETECTION_SENSOR_APP: return "DETECT";
        default:                                     return "UNK";
    }
}

String UARTBridgeModule::extractPayloadText(const meshtastic_MeshPacket &mp)
{
    // Solo extraemos texto de paquetes TEXT_MESSAGE.
    // Telemetría y otros van codificados en protobuf — enviamos su hex resumido.
    if (mp.decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
        // El payload es texto plano
        String text = String((const char *)mp.decoded.payload.bytes,
                              mp.decoded.payload.size);
        // Escapar comillas para no romper el JSON
        text.replace("\"", "'");
        text.replace("\n", " ");
        // Limitar a 80 chars para no reventar el payload LoRaWAN
        if (text.length() > 80) {
            text = text.substring(0, 77) + "...";
        }
        return text;
    }

    // Para otros portnums: resumen del tamaño del payload
    char summary[24];
    snprintf(summary, sizeof(summary), "RAW_%uB", mp.decoded.payload.size);
    return String(summary);
}

// ── LED ────────────────────────────────────────────────────────
void UARTBridgeModule::blinkLED(int times, int ms)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(BRIDGE_LED_PIN, HIGH);
        delay(ms);
        digitalWrite(BRIDGE_LED_PIN, LOW);
        if (i < times - 1) delay(ms);
    }
}