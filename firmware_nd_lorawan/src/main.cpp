#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================================================================
// CREDENCIALES OTAA — Gateway Tektelic (Miguel/Oscar)
// ================================================================
static const u1_t PROGMEM DEVEUI[8]  = { 0x00, 0xA1, 0x4B, 0x00, 0x60, 0x70, 0x80, 0x90 };
static const u1_t PROGMEM APPEUI[8]  = { 0x01, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xAB, 0x54 };
static const u1_t PROGMEM APPKEY[16] = {
    0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6, 0x07, 0x18,
    0x29, 0x3A, 0x4B, 0x5C, 0x6D, 0x7E, 0x8F, 0x90
};

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
// PINES UART — recibe JSON desde LilyGO #1 Meshtastic
// ================================================================
#define UART_RX_PIN  12
#define UART_TX_PIN   4
#define UART_BAUD    115200

// ================================================================
// LED y BOTÓN
// ================================================================
#define LED_PIN    25
#define BTN_PIN     0   // GPIO0 = BOOT button (pull-up interno)

// ================================================================
// OLED SSD1306 — 128x64, I2C (SDA=21, SCL=22)
// ================================================================
#define OLED_W       128
#define OLED_H        64
#define OLED_RESET    -1   // sin pin reset separado
#define OLED_ADDR   0x3C

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, OLED_RESET);

// Carrusel: 4 frames, cambia cada 3 segundos
#define FRAME_COUNT      4
#define FRAME_INTERVAL   3000   // ms

// ================================================================
// VARIABLES GLOBALES
// ================================================================
static bool     joined        = false;
static bool     txPending     = false;
static String   pendingMsg    = "";
static uint32_t pktSent       = 0;     // paquetes enviados al gateway
static uint32_t pktReceived   = 0;     // paquetes recibidos de la mesh
static String   lastPayload   = "---"; // último campo "d" recibido
static int8_t   lastRSSI      = 0;     // último RSSI de TX (downlink)

// Carrusel OLED
static uint8_t  currentFrame  = 0;
static uint32_t lastFrameTime = 0;

// Botón
static bool     btnWasPressed = false;

static osjob_t  sendjob;

// ================================================================
// OLED — Helpers
// ================================================================
void oledTitle(const char *title)
{
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(title);
    // Línea separadora
    display.drawLine(0, 10, OLED_W - 1, 10, SSD1306_WHITE);
}

// ================================================================
// OLED — Frame 0: Identidad del nodo
// ================================================================
void drawFrame0()
{
    display.clearDisplay();
    oledTitle("  NODO DE BORDE");

    display.setTextSize(1);
    display.setCursor(0, 14);
    display.println("  LilyGO LoRaWAN");

    display.setTextSize(1);
    display.setCursor(0, 26);
    display.println("  PFC2 - UTEC");

    display.setCursor(0, 38);
    display.println("  US915 | SF7");

    display.setCursor(0, 50);
    display.printf("  Uptime: %lus", millis() / 1000);

    // Marco decorativo
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

    if (joined) {
        display.println("  JOIN: OK  [TEKTELIC]");
    } else {
        display.println("  JOIN: Buscando...");
    }

    display.setCursor(0, 26);
    display.printf("  TX pend: %s", txPending ? "SI" : "NO");

    display.setCursor(0, 38);
    display.printf("  Enviados: %lu", pktSent);

    display.setCursor(0, 50);
    display.printf("  Recibidos: %lu", pktReceived);

    display.display();
}

// ================================================================
// OLED — Frame 2: UART / Mesh
// ================================================================
void drawFrame2()
{
    display.clearDisplay();
    oledTitle("  UART - MESH");

    display.setTextSize(1);
    display.setCursor(0, 14);
    display.printf("  Rx UART: %lu pkts", pktReceived);

    display.setCursor(0, 26);
    display.println("  Ultimo dato:");

    // Mostrar hasta 20 chars del último payload
    String preview = lastPayload;
    if (preview.length() > 20) preview = preview.substring(0, 17) + "...";

    display.setCursor(4, 38);
    display.println(preview);

    display.setCursor(0, 50);
    display.printf("  UART: GPIO%d RX", UART_RX_PIN);

    display.display();
}

// ================================================================
// OLED — Frame 3: Credenciales (DevEUI resumido)
// ================================================================
void drawFrame3()
{
    display.clearDisplay();
    oledTitle("  CREDENCIALES");

    display.setTextSize(1);

    // Mostrar DevEUI en MSB (invertido del array LSB)
    display.setCursor(0, 14);
    display.println("  DevEUI:");
    display.setCursor(4, 24);
    display.printf("%02X%02X%02X%02X",
        pgm_read_byte(&DEVEUI[7]),
        pgm_read_byte(&DEVEUI[6]),
        pgm_read_byte(&DEVEUI[5]),
        pgm_read_byte(&DEVEUI[4]));
    display.setCursor(4, 34);
    display.printf("%02X%02X%02X%02X",
        pgm_read_byte(&DEVEUI[3]),
        pgm_read_byte(&DEVEUI[2]),
        pgm_read_byte(&DEVEUI[1]),
        pgm_read_byte(&DEVEUI[0]));

    display.setCursor(0, 46);
    display.println("  Region: US915");

    display.setCursor(0, 56);
    display.printf("  Modo: OTAA ClassA");

    display.display();
}

// ================================================================
// OLED — Actualizar carrusel
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
    }
}

// ================================================================
// OLED — Pantalla de boot
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
// OLED — Pantalla de join exitoso (se muestra 2s)
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
// BOTÓN — Detectar pulsación y resetear
// ================================================================
void checkButton()
{
    bool btnNow = (digitalRead(BTN_PIN) == LOW);  // pull-up: LOW = presionado

    if (btnNow && !btnWasPressed) {
        btnWasPressed = true;

        // Mostrar mensaje de reset en OLED
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(20, 20);
        display.println("Reiniciando...");
        display.setCursor(25, 36);
        display.println("por boton");
        display.display();

        Serial.println("[BTN] Boton presionado — reiniciando ESP32");
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
// TRANSMISIÓN LMIC
// ================================================================
void do_send(osjob_t *j)
{
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("[TX] TX en curso, esperando...");
        return;
    }
    if (pendingMsg.length() == 0) return;

    uint8_t payload[220];
    size_t  len = pendingMsg.length();
    if (len > sizeof(payload)) len = sizeof(payload);

    memcpy(payload, pendingMsg.c_str(), len);
    LMIC_setTxData2(1, payload, len, 0);

    Serial.printf("[TX] Enviando %u bytes\n", len);
    Serial.println("[TX] " + pendingMsg);

    txPending  = true;
    pendingMsg = "";
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
            joined = true;
            digitalWrite(LED_PIN, LOW);
            LMIC_setLinkCheckMode(0);
            drawJoinOK();
            // Forzar mostrar frame de estado inmediatamente
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
            blinkLED(1, 300);
            if (LMIC.txrxFlags & TXRX_ACK)
                Serial.println("[LMIC] ACK recibido");
            if (LMIC.dataLen)
                Serial.printf("[LMIC] Downlink: %d bytes\n", LMIC.dataLen);
            break;

        case EV_LINK_DEAD:
            Serial.println("[LMIC] Link muerto — reiniciando join");
            joined = false;
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

    // LED y botón
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    pinMode(BTN_PIN, INPUT_PULLUP);

    // OLED
    Wire.begin(21, 22);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] ERROR — pantalla no encontrada");
    } else {
        Serial.println("[OLED] Pantalla inicializada");
        drawBoot();
        delay(1500);
    }

    // UART para recibir de LilyGO #1
    Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[UART] Serial2 listo RX=GPIO%d TX=GPIO%d\n",
                  UART_RX_PIN, UART_TX_PIN);

    // LMIC
    os_init();
    LMIC_reset();

    // US915 — subband 2 (canales 8-15), probado con Tektelic
    for (uint8_t i = 0; i < 72; i++) {
        if (i < 8 || i > 15) LMIC_disableChannel(i);
    }

    LMIC_setDrTxpow(DR_SF7, 10);
    LMIC_setAdrMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

    Serial.println("[LMIC] Iniciando join OTAA...");
    LMIC_startJoining();

    // Mostrar primer frame
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
    // Motor LMIC — siempre primero
    os_runloop_once();

    // Leer UART desde LilyGO #1
    if (Serial2.available()) {
        String line = Serial2.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) return;

        Serial.println("[UART] Recibido: " + line);
        pktReceived++;

        // Extraer campo "d" del JSON para mostrar en OLED
        int dIdx = line.indexOf("\"d\":\"");
        if (dIdx >= 0) {
            int dStart = dIdx + 5;
            int dEnd   = line.indexOf("\"}", dStart);
            if (dEnd < 0) dEnd = line.indexOf("\"", dStart);
            if (dEnd > dStart)
                lastPayload = line.substring(dStart, dEnd);
        }

        if (!joined) {
            Serial.println("[UART] Sin join aún, descartando");
            blinkLED(3, 80);
            return;
        }

        if (txPending) {
            Serial.println("[UART] TX en curso, descartando");
            blinkLED(2, 80);
            return;
        }

        if (line[0] != '{') {
            Serial.println("[UART] No es JSON, descartando");
            return;
        }

        pendingMsg = line;
        os_setCallback(&sendjob, do_send);
    }

    // Carrusel OLED
    updateOLED();

    // Botón reset
    checkButton();
}