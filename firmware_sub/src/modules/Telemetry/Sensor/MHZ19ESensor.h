#pragma once
#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "TelemetrySensor.h"

// ── Selección de modo ──────────────────────────────────────────
// Comenta/descomenta según el modo que quieras usar
#define CO2_USE_UART      // ← UART activo
//#define CO2_USE_PWM    // ← PWM activo (comenta UART y descomenta esto)

// ── Pines UART (nueva PCB) ─────────────────────────────────────
#define CO2_RX_PIN      34   // GPIO34 ← Tx del sensor (input-only)
#define CO2_TX_PIN      4    // GPIO4  → Rx del sensor
#define CO2_SERIAL_BAUD 9600

// ── Pin PWM (respaldo) ─────────────────────────────────────────
#define CO2_PWM_PIN     35   // GPIO35 ← PWM del sensor (input-only)

// ── Rango y umbrales ───────────────────────────────────────────
#define CO2_RANGE       5000  // MH-Z19E 400-5000ppm
#define CO2_NORMAL_MAX  1000  // ppm normal
#define CO2_WARNING_MAX 2000  // ppm advertencia

// id sensor
#define CO2_NODE_ID  "N2"   

// ── Validación ─────────────────────────────────────────────────
#if defined(CO2_USE_UART) && defined(CO2_USE_PWM)
  #error "Solo puedes definir CO2_USE_UART o CO2_USE_PWM, no ambos"
#endif
#if !defined(CO2_USE_UART) && !defined(CO2_USE_PWM)
  #error "Debes definir CO2_USE_UART o CO2_USE_PWM"
#endif

#ifdef CO2_USE_UART
#if __has_include(<MHZ19.h>)
#include <MHZ19.h>
#else
#error "CO2_USE_UART requiere la libreria WifWaf/MH-Z19 en platformio.ini"
#endif
#endif

class MHZ19ESensor : public TelemetrySensor
{
  private:
#ifdef CO2_USE_UART
    MHZ19 co2sensor;
    HardwareSerial co2Serial;
#endif
    int readPWM();
    int lastPPM = 0;

  public:
    MHZ19ESensor();
    virtual int32_t runOnce() override;
    virtual bool getMetrics(meshtastic_Telemetry *measurement) override;
    int getCO2PPM();
    int getLastPPM();
    bool isCritical();
    bool isWarning();
};

#endif