#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_ENVIRONMENTAL_SENSOR

#include "../mesh/generated/meshtastic/telemetry.pb.h"
#include "MHZ19ESensor.h"
#include "TelemetrySensor.h"

#ifdef CO2_USE_UART
MHZ19ESensor::MHZ19ESensor()
    : TelemetrySensor(meshtastic_TelemetrySensorType_SCD30, "MHZ19E"),
      co2Serial(1)
{
}
#else
MHZ19ESensor::MHZ19ESensor()
    : TelemetrySensor(meshtastic_TelemetrySensorType_SCD30, "MHZ19E")
{
}
#endif

int32_t MHZ19ESensor::runOnce()
{
#ifdef CO2_USE_UART
    LOG_INFO("MHZ19E: Inicializando via UART RX=%d TX=%d\n", CO2_RX_PIN, CO2_TX_PIN);
    co2Serial.begin(CO2_SERIAL_BAUD, SERIAL_8N1, CO2_RX_PIN, CO2_TX_PIN);
    co2sensor.begin(co2Serial);
    co2sensor.autoCalibration(false);
    bool abcStatus = co2sensor.getABC();
    LOG_INFO("MHZ19E: ABC status = %s\n", abcStatus ? "ON (PROBLEMA)" : "OFF (OK)");
    LOG_INFO("MHZ19E: Listo. Rango=%d ppm, offset=%d\n", CO2_RANGE, CO2_CALIBRATION_OFFSET);
#else
    LOG_INFO("MHZ19E: Inicializando via PWM pin=%d\n", CO2_PWM_PIN);
    pinMode(CO2_PWM_PIN, INPUT);
    LOG_INFO("MHZ19E: Listo. Rango=%d ppm\n", CO2_RANGE);
#endif
    return 30000;
}

int MHZ19ESensor::readPWM()
{
    unsigned long start = millis();
    while (digitalRead(CO2_PWM_PIN) == HIGH) {
        if (millis() - start > 1500) return 0;
    }
    while (digitalRead(CO2_PWM_PIN) == LOW) {
        if (millis() - start > 1500) return 0;
    }
    unsigned long th_start = millis();
    while (digitalRead(CO2_PWM_PIN) == HIGH) {
        if (millis() - th_start > 1200) return 0;
    }
    unsigned long th = millis() - th_start;

    unsigned long tl_start = millis();
    while (digitalRead(CO2_PWM_PIN) == LOW) {
        if (millis() - tl_start > 1200) return 0;
    }
    unsigned long tl = millis() - tl_start;

    if (th == 0 || tl == 0) {
        LOG_WARN("MHZ19E: PWM timeout pin %d\n", CO2_PWM_PIN);
        return 0;
    }

    int ppm = (int)((float)CO2_RANGE * (th - 2.0f) / (th + tl - 4.0f));
    LOG_DEBUG("MHZ19E: TH=%lums TL=%lums ppm=%d\n", th, tl, ppm);
    return ppm;
}

bool MHZ19ESensor::getMetrics(meshtastic_Telemetry *measurement)
{
    int ppm = getCO2PPM();
    if (ppm < 100 || ppm > 10000) {
        LOG_WARN("MHZ19E: Lectura invalida: %d ppm\n", ppm);
        return false;
    }
    // Sin offset aqui — el offset lo aplica CO2AlertModule
    if (isCritical())
        LOG_WARN("MHZ19E: CO2=%d ppm (raw) [CRITICO]\n", ppm);
    else if (isWarning())
        LOG_INFO("MHZ19E: CO2=%d ppm (raw) [ADVERTENCIA]\n", ppm);
    else
        LOG_INFO("MHZ19E: CO2=%d ppm (raw) [NORMAL]\n", ppm);

    measurement->variant.environment_metrics.has_temperature = true;
    measurement->variant.environment_metrics.temperature = (float)ppm;
    return true;
}

int MHZ19ESensor::getCO2PPM()
{
#ifdef CO2_USE_UART
    lastPPM = co2sensor.getCO2();
    LOG_DEBUG("MHZ19E: UART ppm=%d\n", lastPPM);
#else
    lastPPM = readPWM();
#endif
    return lastPPM;
}

int MHZ19ESensor::getCO2PPMCorregido()
{
    return lastPPM + CO2_CALIBRATION_OFFSET;
}

int MHZ19ESensor::getLastPPM()
{
    return lastPPM;
}

bool MHZ19ESensor::isCritical()
{
    return (lastPPM + CO2_CALIBRATION_OFFSET) >= CO2_WARNING_MAX;
}

bool MHZ19ESensor::isWarning()
{
    int corregido = lastPPM + CO2_CALIBRATION_OFFSET;
    return (corregido >= CO2_NORMAL_MAX && corregido < CO2_WARNING_MAX);
}

#endif