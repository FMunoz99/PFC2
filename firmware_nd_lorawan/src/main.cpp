#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
// CREDENCIALES OTAA — Gateway Tektelic
// ================================================================
static const u1_t PROGMEM DEVEUI[8]  = { 0x08, 0x07, 0xF6, 0xE5, 0xD4, 0xC3, 0xB2, 0xA1 };
static const u1_t PROGMEM APPEUI[8]  = { 0x80, 0x70, 0x60, 0x50, 0x40, 0x30, 0x20, 0x10 };
static const u1_t PROGMEM APPKEY[16] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
                                          0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99 };

void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

// ================================================================
// PINES LoRa — LilyGO T3_V1.6.1 (NO MODIFICAR)
// ================================================================
const lmic_pinmap lmic_pins = {
    .nss  = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = 14,
    .dio  = {26, 33, 32},
};

// ================================================================
// PINES UART
// ================================================================
#define UART_RX_PIN   34
#define UART_TX_PIN    4
#define UART_BAUD     115200

// ================================================================
// LED y BOTÓN
// ================================================================
#define LED_PIN    25
#define BTN_PIN     0

// ================================================================
// OLED SSD1306 — 128x64, I2C (SDA=21, SCL=22)
// ================================================================
#define OLED_W      128
#define OLED_H       64
#define OLED_RESET   -1
#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

#define FRAME_COUNT      5
#define FRAME_INTERVAL   3000

// ================================================================
// Detección de gateway caído
// ----------------------------------------------------------------
// Los paquetes críticos se envían con confirmación (ACK requerido).
// Si el gateway no responde MAX_TX_FAILURES veces seguidas,
// se considera el link perdido y se reinicia el join OTAA.
// Los paquetes normales siguen siendo no confirmados (fire-and-forget)
// para minimizar latencia y consumo en operación normal.
// ================================================================
#define MAX_TX_FAILURES  3

// ================================================================
// VARIABLES GLOBALES
// ================================================================
static bool     joined              = false;
static bool     txPending           = false;
static bool     linkOK              = false;  // estado real del link
static uint8_t  txConfirmedFailures = 0;      // fallos consecutivos de ACK

// Cola de prioridad
static String   pendingMsgCritico   = "";
static String   pendingMsgNormal    = "";

static uint32_t pktSent             = 0;
static uint32_t pktReceived         = 0;
static uint32_t pktDescartados      = 0;
static String   lastPayload         = "---";

// Métricas del último paquete
static int32_t  lastRSSI            = 0;
static float    lastSNR             = 0.0f;
static int32_t  lastCO2             = 0;
static uint32_t lastTimestamp       = 0;
static bool     lastTxOK            = false;
static bool     lastWasCritico      = false;

// Carrusel OLED
static uint8_t  currentFrame        = 0;
static uint32_t lastFrameTime       = 0;

// Botón
static bool     btnWasPressed       = false;

static osjob_t  sendjob;

// ================================================================
// OLED — Helper título
// ================================================================
void oledTitle(const char *title)
{
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(title);
    display.drawLine(0, 10, OLED_W - 1, 10, SSD1306_WHITE);
}

// ================================================================
// OLED — Frame 0: Identidad
// ================================================================
void drawFrame0()
{
    display.clearDisplay();
    oledTitle("  NODO DE BORDE");

    display.setTextSize(1);
    display.setCursor(0, 14);
    display.println("  LilyGO LoRaWAN");

    display.setCursor(0, 26);
    display.println("  PFC2 - UTEC");

    display.setCursor(0, 38);
    display.println("  AU915 | SF7");

    display.setCursor(0, 50);
    display.printf("  Uptime: %lus", millis() / 1000);

    display.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);
    display.display();
}

// ================================================================
// OLED — Frame 1: Estado LoRaWAN
// ================================================================
void drawFrame1()
{
    display.clearDisplay();
    oledTitle("  ESTADO LORAWAN");

    display.setTextSize(1);
    display.setCursor(0, 14);

    // Estado más preciso — joined Y linkOK
    if (joined && linkOK) {
        display.println("  JOIN: OK [TEKTELIC]");
    } else if (joined && !linkOK) {
        display.println("  JOIN: SIN RESPUESTA");  // gateway caído
    } else {
        display.println("  JOIN: Buscando...");
    }

    display.setCursor(0, 26);
    display.printf("  TX pend: %s", txPending ? "SI" : "NO");

    display.setCursor(0, 38);
    display.printf("  Enviados:  %lu", pktSent);

    display.setCursor(0, 50);
    display.printf("  Recibidos: %lu", pktReceived);

    display.display();
}

// ================================================================
// OLED — Frame 2: Último paquete
// ================================================================
void drawFrame2()
{
    display.clearDisplay();
    oledTitle("  ULTIMO PAQUETE");

    display.setTextSize(1);

    display.setCursor(0, 14);
    if (lastCO2 > 0) {
        display.printf("  CO2: %d ppm", lastCO2);
    } else {
        String preview = lastPayload;
        if (preview.length() > 18) preview = preview.substring(0, 15) + "...";
        display.print("  "); display.println(preview);
    }

    display.setCursor(0, 26);
    display.printf("  RSSI: %d dBm", lastRSSI);

    display.setCursor(0, 38);
    display.printf("  SNR:  %.1f dB", lastSNR);

    display.setCursor(0, 50);
    if (lastTimestamp > 0) {
        uint32_t t  = lastTimestamp;
        uint8_t  hh = (t % 86400) / 3600;
        uint8_t  mm = (t % 3600)  / 60;
        uint8_t  ss =  t % 60;
        display.printf("  %02d:%02d:%02d UTC", hh, mm, ss);
    } else {
        display.println("  ts: ---");
    }

    display.display();
}

// ================================================================
// OLED — Frame 3: Credenciales
// ================================================================
void drawFrame3()
{
    display.clearDisplay();
    oledTitle("  CREDENCIALES");

    display.setTextSize(1);
    display.setCursor(0, 14);
    display.println("  DevEUI:");
    display.setCursor(4, 24);
    display.printf("%02X%02X%02X%02X",
        pgm_read_byte(&DEVEUI[7]), pgm_read_byte(&DEVEUI[6]),
        pgm_read_byte(&DEVEUI[5]), pgm_read_byte(&DEVEUI[4]));
    display.setCursor(4, 34);
    display.printf("%02X%02X%02X%02X",
        pgm_read_byte(&DEVEUI[3]), pgm_read_byte(&DEVEUI[2]),
        pgm_read_byte(&DEVEUI[1]), pgm_read_byte(&DEVEUI[0]));

    display.setCursor(0, 46);
    display.println("  Region: AU915");
    display.setCursor(0, 56);
    display.println("  Modo: OTAA ClassA");

    display.display();
}

// ================================================================
// OLED — Frame 4: Contadores y estado del link
// ================================================================
void drawFrame4()
{
    display.clearDisplay();
    oledTitle("  CONTADORES");

    display.setTextSize(1);

    display.setCursor(0, 14);
    display.printf("  Enviados:  %lu", pktSent);

    display.setCursor(0, 26);
    display.printf("  Recibidos: %lu", pktReceived);

    display.setCursor(0, 38);
    display.printf("  Descart.:  %lu", pktDescartados);

    display.setCursor(0, 50);
    if (pendingMsgCritico.length() > 0) {
        display.println("  CRITICO en cola!");
    } else if (!linkOK && joined) {
        display.printf("  Fallos ACK: %d/%d", txConfirmedFailures, MAX_TX_FAILURES);
    } else if (lastTxOK) {
        display.printf("  UltimoTX: %s",
            lastWasCritico ? "CRITICO OK" : "NORMAL OK");
    } else {
        display.println("  Ultimo TX: ---");
    }

    display.display();
}

// ================================================================
// OLED — Carrusel
// ================================================================
void updateOLED()
{
    uint32_t now = millis();
    if (now - lastFrameTime < FRAME_INTERVAL) return;

    lastFrameTime = now;
    currentFrame  = (currentFrame + 1) % FRAME_COUNT;

    switch (currentFrame) {
        case 0: drawFrame0(); break;
        case 1: drawFrame1(); break;
        case 2: drawFrame2(); break;
        case 3: drawFrame3(); break;
        case 4: drawFrame4(); break;
    }
}

// ================================================================
// OLED — Boot
// ================================================================
void drawBoot()
{
    display.clearDisplay();
    display.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 8);
    display.println("  NODO DE BORDE");

    display.setTextSize(2);
    display.setCursor(14, 22);
    display.println("LoRaWAN");

    display.setTextSize(1);
    display.setCursor(10, 44);
    display.println("  PFC2 - UTEC 2026");
    display.setCursor(10, 54);
    display.println("  Iniciando...");

    display.display();
}

// ================================================================
// OLED — Join OK
// ================================================================
void drawJoinOK()
{
    display.clearDisplay();
    display.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(20, 10);
    display.println("JOIN EXITOSO");

    display.setTextSize(2);
    display.setCursor(30, 26);
    display.println("OK!");

    display.setTextSize(1);
    display.setCursor(10, 50);
    display.println("Gateway: TEKTELIC");

    display.display();
    delay(2000);
}

// ================================================================
// OLED — Gateway caído (se muestra brevemente)
// ================================================================
void drawLinkDead()
{
    display.clearDisplay();
    display.drawRect(0, 0, OLED_W, OLED_H, SSD1306_WHITE);

    display.setTextSize(1);
    display.setCursor(15, 8);
    display.println("GATEWAY CAIDO");

    display.setTextSize(1);
    display.setCursor(10, 24);
    display.println("Sin respuesta ACK");

    display.setCursor(10, 36);
    display.printf("%d fallos consecutivos", MAX_TX_FAILURES);

    display.setCursor(10, 50);
    display.println("Reintentando join...");

    display.display();
    delay(3000);
}

// ================================================================
// BOTÓN
// ================================================================
void checkButton()
{
    bool btnNow = (digitalRead(BTN_PIN) == LOW);

    if (btnNow && !btnWasPressed) {
        btnWasPressed = true;

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(20, 20);
        display.println("Reiniciando...");
        display.setCursor(25, 36);
        display.println("por boton");
        display.display();

        Serial.println("[BTN] Reiniciando ESP32");
        delay(800);
        ESP.restart();
    }

    if (!btnNow) btnWasPressed = false;
}

// ================================================================
// LED
// ================================================================
void blinkLED(int times, int ms)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(ms);
        digitalWrite(LED_PIN, LOW);
        if (i < times - 1) delay(ms);
    }
}

// ================================================================
// TRANSMISIÓN LMIC — cola de prioridad
// ================================================================
void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("[TX] LMIC ocupado, esperando...");
        return;
    }

    String msgToSend  = "";
    uint8_t confirmado = 0;

    if (pendingMsgCritico.length() > 0) {
        msgToSend         = pendingMsgCritico;
        pendingMsgCritico = "";
        lastWasCritico    = true;
        confirmado        = 1;   // ← CONFIRMADO: gateway debe responder con ACK
        Serial.println("[TX] PRIORIDAD ALTA — enviando CRITICO (confirmado)");
    } else if (pendingMsgNormal.length() > 0) {
        msgToSend        = pendingMsgNormal;
        pendingMsgNormal = "";
        lastWasCritico   = false;
        confirmado       = 0;   // ← NO confirmado: fire-and-forget
        Serial.println("[TX] Enviando paquete NORMAL (no confirmado)");
    } else {
        return;
    }

    uint8_t payload[220];
    size_t  len = msgToSend.length();
    if (len > sizeof(payload)) len = sizeof(payload);

    memcpy(payload, msgToSend.c_str(), len);
    LMIC_setTxData2(3, payload, len, confirmado);

    Serial.printf("[TX] Enviando %u bytes\n", len);
    Serial.println("[TX] " + msgToSend);

    txPending = true;
}

// ================================================================
// EVENTO LMIC
// ================================================================
void onEvent(ev_t ev)
{
    switch (ev) {
        case EV_JOINING:
            Serial.println("[LMIC] Intentando join OTAA...");
            digitalWrite(LED_PIN, HIGH);
            break;

        case EV_JOINED:
            Serial.println("[LMIC] ✓ Join exitoso con gateway Tektelic");
            joined              = true;
            linkOK              = true;
            txConfirmedFailures = 0;
            digitalWrite(LED_PIN, LOW);
            LMIC_setLinkCheckMode(0);
            drawJoinOK();
            currentFrame  = 1;
            lastFrameTime = millis() - FRAME_INTERVAL;
            break;

        case EV_JOIN_FAILED:
            Serial.println("[LMIC] ✗ Join fallido — reintentando");
            break;

        case EV_TXCOMPLETE:
            txPending = false;
            pktSent++;
            Serial.printf("[LMIC] TX completo — total enviados: %lu\n", pktSent);

            // ── Verificar ACK para paquetes críticos confirmados ──
            if (lastWasCritico) {
                if (LMIC.txrxFlags & TXRX_ACK) {
                    // Gateway respondió — link OK
                    txConfirmedFailures = 0;
                    linkOK  = true;
                    lastTxOK = true;
                    blinkLED(3, 100);  // triple parpadeo = TX OK con ACK
                    Serial.println("[LMIC] ACK recibido — gateway activo");
                } else {
                    // No hubo ACK — posible gateway caído
                    txConfirmedFailures++;
                    lastTxOK = false;
                    blinkLED(2, 50);   // doble parpadeo rápido = sin ACK
                    Serial.printf("[LMIC] Sin ACK — fallo %d/%d\n",
                                  txConfirmedFailures, MAX_TX_FAILURES);

                    if (txConfirmedFailures >= MAX_TX_FAILURES) {
                        // Gateway definitivamente caído — reiniciar join
                        Serial.println("[LMIC] Gateway no responde — link perdido");
                        joined  = false;
                        linkOK  = false;
                        txConfirmedFailures = 0;
                        drawLinkDead();
                        LMIC_startJoining();
                        return;
                    }
                }
            } else {
                // Paquete normal no confirmado — asumir OK
                lastTxOK = true;
                blinkLED(3, 100);
            }

            if (LMIC.dataLen)
                Serial.printf("[LMIC] Downlink: %d bytes\n", LMIC.dataLen);

            // Enviar siguiente en cola si existe
            if (pendingMsgCritico.length() > 0) {
                Serial.println("[TX] Crítico en cola — enviando inmediatamente");
                os_setCallback(&sendjob, do_send);
            } else if (pendingMsgNormal.length() > 0) {
                os_setCallback(&sendjob, do_send);
            }
            break;

        case EV_LINK_DEAD:
            // Este evento también puede dispararse por LMIC internamente
            Serial.println("[LMIC] EV_LINK_DEAD — gateway sin respuesta");
            joined  = false;
            linkOK  = false;
            txConfirmedFailures = 0;
            drawLinkDead();
            LMIC_startJoining();
            break;

        default:
            Serial.printf("[LMIC] Evento: %d\n", (int)ev);
            break;
    }
}

// ================================================================
// SETUP
// ================================================================
void setup()
{
    Serial.begin(115200);
    Serial.println("\n[BOOT] Nodo de Borde — LilyGO #2 LoRaWAN");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(BTN_PIN, INPUT_PULLUP);

    Wire.begin(21, 22);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] ERROR — pantalla no encontrada");
    } else {
        Serial.println("[OLED] Pantalla inicializada");
        drawBoot();
        delay(1500);
    }

    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[UART] Serial2 listo RX=GPIO%d TX=GPIO%d\n",
                  UART_RX_PIN, UART_TX_PIN);

    os_init();
    LMIC_reset();

    // AU915 — subband 2 (canales 8-15), probado con Tektelic
    for (uint8_t i = 0; i < 72; i++) {
        if (i < 8 || i > 15) LMIC_disableChannel(i);
    }

    LMIC_setDrTxpow(DR_SF7, 10);
    LMIC_setAdrMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    Serial.println("[LMIC] Iniciando join OTAA...");
    LMIC_startJoining();

    drawFrame0();
    lastFrameTime = millis();

    digitalWrite(LED_PIN, LOW);
    Serial.println("[BOOT] Setup completo\n");
}

// ================================================================
// LOOP
// ================================================================
void loop()
{
    os_runloop_once();

    if (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) return;

        Serial.println("[UART] Recibido: " + line);
        pktReceived++;

        // Extraer métricas
        int dIdx = line.indexOf("\"d\":\"");
        if (dIdx >= 0) {
            int dStart = dIdx + 5;
            int dEnd   = line.indexOf("\"}", dStart);
            if (dEnd < 0) dEnd = line.indexOf("\"", dStart);
            if (dEnd > dStart) {
                lastPayload = line.substring(dStart, dEnd);
                int co2Idx  = lastPayload.indexOf("CO2: ");
                if (co2Idx >= 0)
                    lastCO2 = lastPayload.substring(co2Idx + 5).toInt();
            }
        }

        int rsIdx = line.indexOf("\"rs\":");
        if (rsIdx >= 0) lastRSSI = line.substring(rsIdx + 5).toInt();

        int snIdx = line.indexOf("\"sn\":");
        if (snIdx >= 0) lastSNR = line.substring(snIdx + 5).toFloat();

        int tsIdx = line.indexOf("\"ts\":");
        if (tsIdx >= 0) lastTimestamp = (uint32_t)line.substring(tsIdx + 5).toInt();

        bool esCritico = (line.indexOf("\"pr\":\"HIGH\"") >= 0);

        if (!joined) {
            Serial.println("[UART] Sin join aún, descartando");
            blinkLED(3, 80);
            return;
        }

        if (line[0] != '{') {
            Serial.println("[UART] No es JSON, descartando");
            return;
        }

        // Cola de prioridad
        if (esCritico) {
            pendingMsgCritico = line;
            Serial.println("[UART] CRITICO guardado en cola de prioridad");
            if (!txPending) {
                os_setCallback(&sendjob, do_send);
            }
        } else {
            if (txPending) {
                pktDescartados++;
                Serial.printf("[UART] Normal descartado (TX ocupado) — total: %lu\n",
                              pktDescartados);
                return;
            }
            pendingMsgNormal = line;
            os_setCallback(&sendjob, do_send);
        }
    }

    updateOLED();
    checkButton();
}