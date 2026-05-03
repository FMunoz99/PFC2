#pragma once
#include "configuration.h"
#include "MeshModule.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

/**
 * UARTBridgeModule — Nodo de Borde, LilyGO #1
 * -----------------------------------------------
 * Escucha TODOS los paquetes que llegan a la mesh
 * y los reenvía por Serial2 (UART) al LilyGO #2
 * en formato JSON compacto compatible con LoRaWAN.
 *
 * Pines UART (LilyGO T3_V1.6.1):
 *   GPIO12 → TX hacia LilyGO #2 RX
 *   GPIO34 → RX desde LilyGO #2 TX (input-only, no usado activamente)
 *
 * LED de estado:
 *   GPIO25 → LED built-in (parpadeo rápido = paquete recibido/enviado)
 */

// ── Pines ──────────────────────────────────────────────────────
#define BRIDGE_UART_TX_PIN   4    // GPIO4 → TX hacia LilyGO #2
#define BRIDGE_UART_RX_PIN   34   // GPIO34 → RX desde LilyGO #2 (input-only)
#define BRIDGE_UART_BAUD     115200
#define BRIDGE_LED_PIN       25   // LED built-in

// ── Límite de payload ──────────────────────────────────────────
// US915 SF7 → máx 242 bytes. Dejamos margen para overhead LoRaWAN.
#define BRIDGE_MAX_PAYLOAD   220

class UARTBridgeModule : public MeshModule
{
  public:
    UARTBridgeModule();

    // Obligatorio — acepta TODOS los paquetes
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override { return true; }

    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    bool uartReady = false;
    void initUART();
    void blinkLED(int times = 1, int ms = 100);
    bool sendPacketUART(const meshtastic_MeshPacket &mp);
    String portNumToString(meshtastic_PortNum port);
    String extractPayloadText(const meshtastic_MeshPacket &mp);
};

extern UARTBridgeModule *uartBridgeModule;