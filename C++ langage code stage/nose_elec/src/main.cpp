/*******************************************************
 * WIO_BLE_Streamer_rpcBLE_NUS.ino
 * Wio Terminal — BLE (rpcBLE API) streaming 5 min @1 Hz
 * Service NUS (TX notify / RX write). PC ➜ bleak ➜ CSV (+timestamps PC).
 *
 * Capteurs:
 *  - BME688: BME_inv = 100000000.0f / gas_resistance  (montée en présence de gaz)
 *  - GasV2: NO2, CO, VOC, Ethanol (bruts)
 *  - SGP40, SGP41: valeurs /100 pour lisibilité
 *  - SGP41: executeConditioning 10 s, puis measureRawSignals
 *
 * TCA9548A canaux: BME=0, Gas=1, SGP40=2, SGP41=3
 *******************************************************/

// --- Base Arduino ---
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// --- BLE d'abord (rpcBLE tire la STL) ---
#include <rpcBLEDevice.h>
#include <BLEServer.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>

// --- Capteurs ---
#include <Adafruit_BME680.h>
#include <Multichannel_Gas_GMXXX.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2CSgp41.h>

// --- TFT EN DERNIER pour éviter de casser la STL avec ses macros ---
#include <TFT_eSPI.h>

// Neutraliser les macros min/max fuyantes de certains headers (ex. TFT_eSPI.h)
#ifdef min
  #undef min
#endif
#ifdef max
  #undef max
#endif

#pragma region THEME
const uint16_t COL_BG       = TFT_BLACK;
const uint16_t COL_CARD_BG  = 0x0841; // dark grey-blue
const uint16_t COL_TITLE_BG = 0x2104; // slightly lighter header
const uint16_t COL_BORDER   = TFT_DARKGREY;
const uint16_t COL_TITLE    = TFT_CYAN;
const uint16_t COL_LABEL    = TFT_WHITE;
const uint16_t COL_VAL      = TFT_GREEN;
const uint16_t COL_WARN     = TFT_ORANGE;
const uint16_t COL_ERR      = TFT_RED;

const int      MARGIN   = 8;
const int      GUTTER   = 8;
const int      HDR_H    = 20; // title bar height
const int      PAD      = 6;  // inner padding in cards
const int      ROW_H    = 18; // per label/value row
#pragma endregion

/* ---------- TCA9548A + canaux ---------- */
#define TCA9548A_ADDR 0x70
#define CHAN_BME    0
#define CHAN_GAS    1
#define CHAN_SGP40  2
#define CHAN_SGP41  3

static inline void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

/* ---------- UI minimal ---------- */
#define SCR_W 320
#define SCR_H 240
#define STATUS_H 18
TFT_eSPI tft;

// MODIF AFFICHAGE: permet de forcer un redessin complet seulement quand nécessaire.
bool dashboardNeedsFullRedraw = true;

void drawStatus(const char* left, const char* right);
void drawDashboardPro(); // declaration ajoutee pour corriger l'erreur

void drawStatus(const char* left, const char* right) {
  tft.fillRect(0, 0, SCR_W, STATUS_H, TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Texte gauche
  tft.setCursor(2, 4);
  tft.print(left);

  // Texte droite : zone propre + alignement à droite pour éviter les mots coupés
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(145);
  tft.drawString(right, SCR_W - 4, 4);
  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
}

/* ---------- Capteurs ---------- */
Adafruit_BME680 bme;
GAS_GMXXX<TwoWire> gas;
SensirionI2CSgp40 sgp40;
SensirionI2CSgp41 sgp41;

bool hasBME=false, hasGAS=false, hasSGP40=false, hasSGP41=false;

/* ---------- Valeurs ---------- */
float vTemp=0, vHum=0, vPress=0, vGasRes=0, vBMEinv=0;
int32_t vNO2=0, vCO=0, vVOC=0, vEth=0;
uint16_t vSGP40=0, vSGP41_VOC=0, vSGP41_NOx=0;

// Ticks fixes (pas de compensation active)
uint16_t defaultRhTicks = 0x8000; // ~50%RH
uint16_t defaultTTicks  = 0x6666; // ~23°C
uint16_t sgp41_conditioning_s = 10; // NOx conditioning comme l'exemple Sensirion
/* ---------- Cadence & session ---------- */
const uint32_t SAMPLE_MS = 1000;
#define BTN_CENTER WIO_5S_PRESS
bool sendingActive=false;
uint32_t sessionStart=0;
const uint32_t SEND_WINDOW_MS = 2400000UL; // 40 minutes
/* ---------- NUS (UUIDs) ---------- */
static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // Notify
static const char* NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  // Write

/* ---------- BLE objets ---------- */
BLEServer*         pServer         = nullptr;
BLEService*        pService        = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
BLECharacteristic* pRxCharacteristic = nullptr;
volatile bool deviceConnected = false;

/* ---------- Helper: éviter le token 'min' ---------- */
static inline size_t ble_min_size(size_t a, size_t b) { return (a < b) ? a : b; }

/* ---------- TX helper : notify avec découpage ---------- */
static const size_t BLE_CHUNK = 180; // payload sûr sans dépendre du MTU négocié
void bleNotifyLine(const String& s) {
  if (!deviceConnected || !pTxCharacteristic) return;
  const char* data = s.c_str();
  size_t len = s.length();
  size_t offset = 0;
  while (offset < len) {
    size_t n = ble_min_size(BLE_CHUNK, len - offset);
    pTxCharacteristic->setValue((uint8_t*)(data + offset), n);
    pTxCharacteristic->notify();
    offset += n;
    delay(1); // petite respiration pour la pile BLE
  }
}

/* ---------- RX callback (commandes START/STOP optionnelles) ---------- */
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;

    String cmd;
    cmd.reserve(v.size());
    for (char ch : v) {
      if (ch >= 'a' && ch <= 'z') cmd += (char)(ch - 32);
      else cmd += ch;
    }
    cmd.trim();

    if (cmd.startsWith("START") && !sendingActive) {
      sendingActive = true;
      sessionStart = millis();
      bleNotifyLine("#START\n");
      bleNotifyLine("t_ms,temp_C,hum_pct,press_hPa,BME_inv,NO2,CO,VOC,Eth,SGP40_div100,SGP41_VOC_div100,SGP41_NOx_div100\n");
      drawStatus("BLE connected", "Sending…");
    } else if (cmd.startsWith("STOP") && sendingActive) {
      sendingActive = false;
      bleNotifyLine("#END\n");
      drawStatus("BLE connected", "Idle");
    }
  }
};

/* ---------- Server callbacks : connexion ---------- */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    drawStatus("BLE connected", sendingActive ? "Sending…" : "Connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    sendingActive = false; // sécurité
    drawStatus("BLE adv: WioENose", "Press CENTER to stream");
    BLEDevice::startAdvertising();
  }
};

/* ---------- Lecture capteurs ---------- */
void readSensorsOnce() {
  // BME
  tcaSelect(CHAN_BME);
  if (hasBME && bme.performReading()) {
    vTemp = bme.temperature;
    vHum  = bme.humidity;
    vPress= bme.pressure/100.0f;
    vGasRes = bme.gas_resistance;
    vBMEinv = (vGasRes > 1.0f) ? (100000000.0f / vGasRes) : 0.0f;
  }
  // Gas V2
  tcaSelect(CHAN_GAS);
  if (hasGAS) {
    vNO2 = gas.getGM102B();
    vEth = gas.getGM302B();
    vVOC = gas.getGM502B();
    vCO  = gas.getGM702B();
  }
  // SGP40
  tcaSelect(CHAN_SGP40);
  if (hasSGP40) {
    (void)sgp40.measureRawSignal(defaultRhTicks, defaultTTicks, vSGP40);
  }
  // SGP41
  tcaSelect(CHAN_SGP41);
  if (hasSGP41) {
    if (sgp41_conditioning_s > 0) {
      (void)sgp41.executeConditioning(defaultRhTicks, defaultTTicks, vSGP41_VOC);
      vSGP41_NOx = 0;
      sgp41_conditioning_s--;
    } else {
      (void)sgp41.measureRawSignals(defaultRhTicks, defaultTTicks, vSGP41_VOC, vSGP41_NOx);
    }
  }
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(BTN_CENTER, INPUT_PULLUP);

  // Écran
  tft.begin(); tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  drawStatus("Boot…", "");

  // Capteurs
  tcaSelect(CHAN_BME);
  hasBME = bme.begin(0x76);
  if (hasBME) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(400, 150);
  }
  tcaSelect(CHAN_GAS);   gas.begin(Wire, 0x08); hasGAS   = true;
  tcaSelect(CHAN_SGP40); sgp40.begin(Wire);     hasSGP40 = true;
  tcaSelect(CHAN_SGP41); sgp41.begin(Wire);     hasSGP41 = true;

  // BLE (rpcBLE)
  BLEDevice::init("WioENose");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  pService = pServer->createService(NUS_SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    NUS_TX_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pTxCharacteristic->addDescriptor(new BLE2902()); // CCCD

  pRxCharacteristic = pService->createCharacteristic(
    NUS_RX_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pRxCharacteristic->setCallbacks(new RxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(NUS_SERVICE_UUID);
  BLEDevice::startAdvertising();

  drawStatus("BLE adv: WioENose", "Press CENTER to stream");
  Serial.println("BLE advertising. Connect from PC and press CENTER for 5 min streaming.");
}

/* ---------- Loop ---------- */
void loop() {
  static uint32_t lastSample = 0;
  const uint32_t now = millis();

  // --- Gestion bouton (démarrage session) ---
  if (deviceConnected && !sendingActive && (digitalRead(BTN_CENTER) == LOW)) {
    delay(20); // debounce
    if (digitalRead(BTN_CENTER) == LOW) {
      sendingActive = true;
      sessionStart = now;
      bleNotifyLine("#START\n");
      bleNotifyLine("t_ms,temp_C,hum_pct,press_hPa,BME_inv,NO2,CO,VOC,Eth,SGP40_div100,SGP41_VOC_div100,SGP41_NOx_div100\n");
      drawStatus("BLE connected", "Sending…");
    }
  }

  // --- Acquisition + affichage toutes les 1 s ---
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    // Lire capteurs
    readSensorsOnce();

    drawDashboardPro();

    // --- Envoi BLE si actif ---
    if (deviceConnected && sendingActive) {
      float s40  = vSGP40 / 100.0f;
      float s41v = vSGP41_VOC / 100.0f;
      float s41n = vSGP41_NOx / 100.0f;

      String line;
      line.reserve(160);
      line += String(now);       line += ",";
      line += String(vTemp, 2);  line += ",";
      line += String(vHum, 2);   line += ",";
      line += String(vPress, 1); line += ",";
      line += String(vBMEinv, 2);line += ",";
      line += String(vNO2);      line += ",";
      line += String(vCO);       line += ",";
      line += String(vVOC);      line += ",";
      line += String(vEth);      line += ",";
      line += String(s40, 2);    line += ",";
      line += String(s41v, 2);   line += ",";
      line += String(s41n, 2);
      line += "\n";
      bleNotifyLine(line);
    }
  }

  // --- Arrêt auto après 5 min ---
  if (sendingActive && (millis() - sessionStart >= SEND_WINDOW_MS)) {
    sendingActive = false;
    bleNotifyLine("#END\n");
    drawStatus(deviceConnected ? "BLE connected" : "BLE adv: WioENose", "Idle");
    Serial.println("Streaming session ended (5 min).");
  }

  delay(2); // respiration
}

// --------- Helpers ----------
void drawCardFrame(int x, int y, int w, int h, const char* title, bool online,
                   int &cx, int &cy, int &cw, int &ch) {
  // Card background & border
  tft.fillRoundRect(x, y, w, h, 6, COL_CARD_BG);
  tft.drawRoundRect(x, y, w, h, 6, COL_BORDER);

  // Title bar
  tft.fillRoundRect(x, y, w, HDR_H, 6, COL_TITLE_BG);
  // square the bottom of header so the round corners remain nice
  tft.fillRect(x, y + HDR_H - 6, w, 6, COL_TITLE_BG);

  // Title text
  tft.setTextFont(2);       // compact readable font
  tft.setTextColor(COL_TITLE, COL_TITLE_BG);
  tft.setTextDatum(TL_DATUM);
  tft.drawString(title, x + PAD, y + 2);

  // Online/Offline indicator (right)
  tft.setTextDatum(TR_DATUM);
  if (online) {
    tft.setTextColor(COL_VAL, COL_TITLE_BG);
    tft.drawString("ONLINE", x + w - PAD, y + 2);
  } else {
    tft.setTextColor(COL_ERR, COL_TITLE_BG);
    tft.drawString("OFFLINE", x + w - PAD, y + 2);
  }

  // Content area
  cx = x + PAD;
  cy = y + HDR_H + PAD;
  cw = w - 2 * PAD;
  ch = h - HDR_H - 2 * PAD;

  // Reset datum for content rows
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2); // keep consistent
  tft.setTextColor(COL_LABEL, COL_CARD_BG);
}

void drawLabelRightValue(int x, int y, int w, const char* label, const String& value,
                         uint16_t labelColor = COL_LABEL, uint16_t valueColor = COL_VAL) {
  // Label on left
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(labelColor, COL_CARD_BG);
  tft.drawString(label, x, y);

  // Value right-aligned
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(valueColor, COL_CARD_BG);

  // Optional padding to avoid ghosting when numbers shorten
  tft.setTextPadding(64); // adjust as needed per max width
  tft.drawString(value, x + w, y);
  tft.setTextPadding(0);

  // Reset datum
  tft.setTextDatum(TL_DATUM);
}

void drawCenteredFail(int cx, int cy, int cw, int ch, const char* msg="NO DATA") {
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(COL_ERR, COL_CARD_BG);
  tft.drawString(msg, cx + cw/2, cy + ch/2 - 4);
  tft.setTextDatum(TL_DATUM);
}

// MODIF AFFICHAGE: version valeurs seules, sans réécrire le label à chaque fois.
void drawValueOnlyRight(int x, int y, int w, const String& value,
                        uint16_t valueColor = COL_VAL) {
  tft.setTextFont(2);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(valueColor, COL_CARD_BG);
  tft.setTextPadding(64);
  tft.drawString(value, x + w, y);
  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
}

// Format helpers
String fmtFloat(float v, uint8_t dec=1) { return String(v, dec); }
String fmtInt(int v) { return String(v); }
#pragma region DEGREE
// Use "°C" if your font supports it; otherwise use " C"
String degC(float v, uint8_t dec=1, bool safeAscii=false) {
  if (safeAscii) return String(v, dec) + " C";
  return String(v, dec) + "°C";
}
#pragma endregion

void drawDashboardPro() {
  // Layout: 2 columns, 2 rows
  const int availH = SCR_H - STATUS_H - 2*MARGIN;
  const int availW = SCR_W - 2*MARGIN;
  const int cardW  = (availW - GUTTER) / 2;
  const int cardH  = (availH - GUTTER) / 2;

  // Card positions
  int x1 = MARGIN;
  int x2 = MARGIN + cardW + GUTTER;
  int y1 = STATUS_H + MARGIN;
  int y2 = STATUS_H + MARGIN + cardH + GUTTER;

  // MODIF AFFICHAGE: on dessine les cartes une seule fois.
  if (dashboardNeedsFullRedraw) {
    tft.fillRect(0, STATUS_H, SCR_W, SCR_H - STATUS_H, COL_BG);

    int cx, cy, cw, ch;
    drawCardFrame(x1, y1, cardW, cardH, "BME688", hasBME, cx, cy, cw, ch);
    drawCardFrame(x2, y1, cardW, cardH, "GasV2", hasGAS, cx, cy, cw, ch);
    drawCardFrame(x1, y2, cardW, cardH, "SGP40", hasSGP40, cx, cy, cw, ch);
    drawCardFrame(x2, y2, cardW, cardH, "SGP41", hasSGP41, cx, cy, cw, ch);

    dashboardNeedsFullRedraw = false;
  }

  // ---------------- BME688 ----------------
  {
    int cx = x1 + PAD;
    int cy = y1 + HDR_H + PAD;
    int cw = cardW - 2 * PAD;
    int ch = cardH - HDR_H - 2 * PAD;

    if (hasBME) {
      int rowY = cy;
      drawLabelRightValue(cx, rowY, cw, "Temp", degC(vTemp, 1));  rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "RH",   fmtFloat(vHum, 1) + " %"); rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "Press", fmtFloat(vPress, 0) + " hPa"); rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "1/gas", fmtFloat(vBMEinv, 1));
    } else {
      drawCenteredFail(cx, cy, cw, ch);
    }
  }

  // ---------------- GasV2 -----------------
  {
    int cx = x2 + PAD;
    int cy = y1 + HDR_H + PAD;
    int cw = cardW - 2 * PAD;
    int ch = cardH - HDR_H - 2 * PAD;

    if (hasGAS) {
      int rowY = cy;
      drawLabelRightValue(cx, rowY, cw, "NO2", fmtFloat(vNO2, 0));  rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "CO",  fmtFloat(vCO, 0));   rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "VOC", fmtFloat(vVOC, 0));  rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "Eth", fmtFloat(vEth, 0));
    } else {
      drawCenteredFail(cx, cy, cw, ch);
    }
  }

  // ---------------- SGP40 -----------------
  {
    int cx = x1 + PAD;
    int cy = y2 + HDR_H + PAD;
    int cw = cardW - 2 * PAD;
    int ch = cardH - HDR_H - 2 * PAD;

    if (hasSGP40) {
      int rowY = cy;
      drawLabelRightValue(cx, rowY, cw, "Raw/100", fmtFloat(vSGP40 / 100.0f, 2));
    } else {
      drawCenteredFail(cx, cy, cw, ch);
    }
  }

  // ---------------- SGP41 -----------------
  {
    int cx = x2 + PAD;
    int cy = y2 + HDR_H + PAD;
    int cw = cardW - 2 * PAD;
    int ch = cardH - HDR_H - 2 * PAD;

    if (hasSGP41) {
      int rowY = cy;
      drawLabelRightValue(cx, rowY, cw, "VOC/100", fmtFloat(vSGP41_VOC / 100.0f, 2)); rowY += ROW_H;
      drawLabelRightValue(cx, rowY, cw, "NOx/100", fmtFloat(vSGP41_NOx / 100.0f, 2));
    } else {
      drawCenteredFail(cx, cy, cw, ch);
    }
  }
}
