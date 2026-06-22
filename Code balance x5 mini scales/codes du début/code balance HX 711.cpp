#include <Arduino.h>
#include <TFT_eSPI.h>
#include <HX711.h>
#include <SPI.h>
#include <SD.h>

// Forward declarations for BLE helpers provided by BLE_Service_Wio.ino
void bleInit();
void bleUpdateWeights(const float* weights, uint8_t count);

// ------------ Hardware configuration ------------

// HX711 wiring: all scales share SCK, each has its own DOUT pin.
// Adjust these pins to match your wiring on the Wio Terminal GPIO header.
static const uint8_t HX_SCK_PIN = 0;
static const uint8_t HX_DOUT_PINS[] = {
  1 , 2 // Single load cell for now; add more pins as needed
};
static const uint8_t NUM_SCALES = sizeof(HX_DOUT_PINS) / sizeof(HX_DOUT_PINS[0]);

// Wio Terminal input pins (defined by board variant)
#ifndef WIO_KEY_A
  #define WIO_KEY_A      0
  #define WIO_KEY_B      1
  #define WIO_KEY_C      2
  #define WIO_5S_UP      3
  #define WIO_5S_DOWN    4
  #define WIO_5S_LEFT    5
  #define WIO_5S_RIGHT   6
  #define WIO_5S_PRESS   7
#endif

// SD card chip select is provided by the variant as SDCARD_SS_PIN
#define SDCARD_SS_PIN SS


// ------------ UI and behavior configuration ------------

static const uint32_t LOG_INTERVAL_MS = 4UL * 60UL * 60UL * 1000UL; // 4 hours
static const char* LOG_FILE = "/weights.csv";
static const char* CALIB_FILE = "/calib.csv";

static const uint16_t SCREEN_BG = TFT_BLACK;
static const uint16_t SCREEN_FG = TFT_WHITE;
static const uint16_t SCREEN_ACCENT = TFT_CYAN;
static const uint16_t SCREEN_WARN = TFT_ORANGE;
static const uint16_t SCREEN_ERR = TFT_RED;

static const uint16_t HEADER_HEIGHT = 22;
static const uint16_t ROW_HEIGHT = 20;
static const uint8_t TEXT_SIZE = 2;

// 100 samples over ~10 seconds per scale
static const uint16_t AVG_SAMPLES = 100;
static const uint16_t AVG_SAMPLE_DELAY_MS = 100; // 100 * 100ms = 10s
// UI averaging for smoother measure display: 100 samples over ~1s
static const uint16_t UI_AVG_SAMPLES = 100;
static const uint16_t UI_SAMPLE_DELAY_MS = 10; // 100 * 10ms = 1s
static const float UI_EMA_ALPHA = 0.2f; // non-blocking smoothing for live display

// ------------ Globals ------------

TFT_eSPI tft;
TFT_eSprite spr = TFT_eSprite(&tft);
HX711 scales[NUM_SCALES];

float calibrationFactors[NUM_SCALES];
long tareOffsets[NUM_SCALES];
float weightsEma[NUM_SCALES];

File fileHandle;
bool sdOk = false;
uint32_t lastLogTimeMs = 0;

enum AppMode { MODE_MEASURE = 0, MODE_CALIBRATE = 1, MODE_SETTINGS = 2 };
AppMode currentMode = MODE_MEASURE;

uint8_t selectedIndex = 0;     // For selection in menus
uint8_t listPage = 0;          // For paginating scale list
uint8_t itemsPerPage = 0;

// Cached input events per loop to avoid multiple update() calls
int8_t evBtnA = 0, evBtnB = 0, evBtnC = 0;
int8_t evUp = 0, evDown = 0, evLeft = 0, evRight = 0, evPress = 0;

// ------------ Debounced input handling ------------

struct DebouncedButton {
  uint8_t pin;
  bool activeLow;
  bool stableState;
  bool lastRead;
  uint32_t lastChangeMs;
  uint32_t repeatStartDelayMs;
  uint32_t repeatRateMs;
  uint32_t lastRepeatFireMs;

  void begin(uint8_t inPin, bool isActiveLow = true, uint32_t debounceMs = 20, uint32_t repeatDelay = 500, uint32_t repeatRate = 200) {
    pin = inPin;
    activeLow = isActiveLow;
    pinMode(pin, INPUT_PULLUP);
    bool raw = digitalRead(pin) == (activeLow ? LOW : HIGH);
    stableState = raw;
    lastRead = raw;
    lastChangeMs = millis();
    repeatStartDelayMs = repeatDelay;
    repeatRateMs = repeatRate;
    lastRepeatFireMs = 0;
    (void)debounceMs; // Debounce fixed at 20ms in update()
  }

  // Returns: 1 on pressed edge or repeat, -1 on released edge, 0 otherwise
  int8_t update() {
    const uint32_t now = millis();
    bool raw = digitalRead(pin) == (activeLow ? LOW : HIGH);
    if (raw != lastRead) {
      lastRead = raw;
      lastChangeMs = now;
    }
    // Debounce 20ms
    if ((now - lastChangeMs) >= 20) {
      if (raw != stableState) {
        stableState = raw;
        if (stableState) {
          lastRepeatFireMs = now;
          return 1; // pressed edge
        } else {
          return -1; // released edge
        }
      } else if (stableState) {
        // Held: handle repeat
        if ((now - lastRepeatFireMs) >= (lastRepeatFireMs == lastChangeMs ? repeatStartDelayMs : repeatRateMs)) {
          lastRepeatFireMs = now;
          return 1; // repeat fire
        }
      }
    }
    return 0;
  }

  bool isPressed() const { return stableState; }
};

DebouncedButton btnA, btnB, btnC;
DebouncedButton joyUp, joyDown, joyLeft, joyRight, joyPress;

// ------------ Utility ------------

void drawHeader(const char* title) {
  spr.fillRect(0, 0, spr.width(), HEADER_HEIGHT, SCREEN_ACCENT);
  spr.setTextColor(TFT_BLACK, SCREEN_ACCENT);
  spr.setTextSize(2);
  spr.setCursor(6, 2);
  spr.print(title);
  spr.setTextColor(SCREEN_FG, SCREEN_BG);
}

void clearContent() {
  spr.fillRect(0, HEADER_HEIGHT, spr.width(), spr.height() - HEADER_HEIGHT, SCREEN_BG);
}

void drawFooter(const char* hint) {
  uint16_t y = spr.height() - 16;
  spr.fillRect(0, y, spr.width(), 16, TFT_DARKGREY);
  spr.setTextColor(TFT_WHITE, TFT_DARKGREY);
  spr.setTextSize(1);
  spr.setCursor(4, y + 4);
  spr.print(hint);
  spr.setTextColor(SCREEN_FG, SCREEN_BG);
}

// Compute items per page based on font size and screen
void computeItemsPerPage() {
  itemsPerPage = (spr.height() - HEADER_HEIGHT - 20) / ROW_HEIGHT;
  if (itemsPerPage < 1) itemsPerPage = 1;
}

// Read a line safely to a buffer
bool readLine(File& f, String& out) {
  out = "";
  while (f.available()) {
    char c = (char)f.read();
    if (c == '\r') continue;
    if (c == '\n') return true;
    out += c;
  }
  return out.length() > 0;
}

// ------------ SD and persistence ------------

bool ensureSD() {
  if (sdOk) return true;
  sdOk = SD.begin(SDCARD_SS_PIN);
  return sdOk;
}

void ensureLogHeader() {
  if (!ensureSD()) return;
  if (!SD.exists(LOG_FILE)) {
    File f = SD.open(LOG_FILE, FILE_WRITE);
    if (f) {
      f.println("timestamp_ms,scale_index,avg_weight");
      f.close();
    }
  }
}

void loadCalibration() {
  for (uint8_t i = 0; i < NUM_SCALES; i++) {
    calibrationFactors[i] = 1.0f;
    tareOffsets[i] = 0;
  }
  if (!ensureSD()) return;
  if (!SD.exists(CALIB_FILE)) return;

  File f = SD.open(CALIB_FILE, FILE_READ);
  if (!f) return;

  String line;
  while (readLine(f, line)) {
    line.trim();
    if (line.length() == 0) continue;
    // Format: index,factor,offset
    int p1 = line.indexOf(',');
    int p2 = (p1 >= 0) ? line.indexOf(',', p1 + 1) : -1;
    if (p1 < 0 || p2 < 0) continue;
    int idx = line.substring(0, p1).toInt();
    float fac = line.substring(p1 + 1, p2).toFloat();
    long off = line.substring(p2 + 1).toInt();
    if (idx >= 0 && idx < NUM_SCALES) {
      calibrationFactors[idx] = fac;
      tareOffsets[idx] = off;
    }
  }
  f.close();
}

void saveCalibration() {
  if (!ensureSD()) return;
  // Overwrite file
  File f = SD.open(CALIB_FILE, FILE_WRITE);
  if (!f) return;
  f.println("index,factor,offset");
  for (uint8_t i = 0; i < NUM_SCALES; i++) {
    f.print(i);
    f.print(",");
    f.print(calibrationFactors[i], 6);
    f.print(",");
    f.println(tareOffsets[i]);
  }
  f.close();
}

// ------------ HX711 helpers ------------

void initScales() {
  for (uint8_t i = 0; i < NUM_SCALES; i++) {
    scales[i].begin(HX_DOUT_PINS[i], HX_SCK_PIN);
    scales[i].set_scale(); // 1.0 default
    scales[i].tare();
    weightsEma[i] = 0.0f;
  }
  // Apply persisted calibration if any
  for (uint8_t i = 0; i < NUM_SCALES; i++) {
    scales[i].set_scale(calibrationFactors[i]);
    if (tareOffsets[i] != 0) {
      // HX711 library typically doesn't expose set_offset; use tare if needed
      // We will re-tare here to keep consistent baseline and then store offset
      scales[i].tare();
      tareOffsets[i] = scales[i].get_offset();
    } else {
      tareOffsets[i] = scales[i].get_offset();
    }
  }
}

float readWeight(uint8_t idx, uint8_t nSamples = 5) {
  return scales[idx].get_units(nSamples);
}

float averageWeightUI(uint8_t idx) {
  float sum = 0.0f;
  for (uint16_t i = 0; i < UI_AVG_SAMPLES; i++) {
    sum += scales[idx].get_units(1);
    delay(UI_SAMPLE_DELAY_MS);
  }
  return sum / (float)UI_AVG_SAMPLES;
}

float averageWeightOverWindow(uint8_t idx) {
  float sum = 0.0f;
  for (uint16_t i = 0; i < AVG_SAMPLES; i++) {
    sum += scales[idx].get_units(1);
    delay(AVG_SAMPLE_DELAY_MS);
  }
  return sum / (float)AVG_SAMPLES;
}

// ------------ Screens ------------

void renderMeasure() {
  drawHeader("Measure");
  clearContent();
  computeItemsPerPage();

  uint16_t y = HEADER_HEIGHT + 2;
  uint8_t start = listPage * itemsPerPage;
  uint8_t end = min<uint8_t>(NUM_SCALES, start + itemsPerPage);

  spr.setTextSize(TEXT_SIZE);
  for (uint8_t i = start; i < end; i++) {
    float w = weightsEma[i];
    if (i == selectedIndex) {
      spr.fillRect(0, y - 2, spr.width(), ROW_HEIGHT, TFT_DARKGREY);
      spr.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    } else {
      spr.setTextColor(SCREEN_FG, SCREEN_BG);
    }
    spr.setCursor(6, y);
    spr.printf("Scale %u: %.2f g", i, w);
    y += ROW_HEIGHT;
  }

  drawFooter("5-way: navigate  A: calibrate  B: tare  C: menu");
  spr.pushSprite(0, 0);
}

void renderCalibrateList() {
  drawHeader("Calibrate");
  clearContent();
  computeItemsPerPage();

  uint16_t y = HEADER_HEIGHT + 2;
  uint8_t start = listPage * itemsPerPage;
  uint8_t end = min<uint8_t>(NUM_SCALES, start + itemsPerPage);

  spr.setTextSize(TEXT_SIZE);
  for (uint8_t i = start; i < end; i++) {
    if (i == selectedIndex) {
      spr.fillRect(0, y - 2, spr.width(), ROW_HEIGHT, TFT_DARKGREY);
      spr.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    } else {
      spr.setTextColor(SCREEN_FG, SCREEN_BG);
    }
    spr.setCursor(6, y);
    spr.printf("Scale %u  2-pt calib", i);
    y += ROW_HEIGHT;
  }
  drawFooter("A: edit  B: tare selected  C: back");
  spr.pushSprite(0, 0);
}

void renderCalibrateEditStep(uint8_t idx, const char* stepTitle, const char* instructions, float previewWeight) {
  drawHeader(stepTitle);
  clearContent();
  spr.setTextSize(TEXT_SIZE);
  spr.setCursor(6, HEADER_HEIGHT + 8);
  spr.setTextColor(SCREEN_FG, SCREEN_BG);
  spr.printf("Scale %u", idx);
  spr.setCursor(6, HEADER_HEIGHT + 8 + ROW_HEIGHT);
  spr.printf("Now: %.2f g", previewWeight);
  spr.setCursor(6, HEADER_HEIGHT + 8 + 3 * ROW_HEIGHT);
  spr.setTextColor(SCREEN_WARN, SCREEN_BG);
  spr.print(instructions);
  drawFooter("A: confirm  B: tare  C: cancel");
  spr.pushSprite(0, 0);
}

void renderMenu() {
  drawHeader("Menu");
  clearContent();
  static const char* items[] = { "Measure", "Calibrate", "Settings" };
  const uint8_t N = 3;
  spr.setTextSize(TEXT_SIZE);
  uint16_t y = HEADER_HEIGHT + 2;
  for (uint8_t i = 0; i < N; i++) {
    if ((uint8_t)currentMode == i) {
      spr.fillRect(0, y - 2, spr.width(), ROW_HEIGHT, TFT_DARKGREY);
      spr.setTextColor(TFT_YELLOW, TFT_DARKGREY);
    } else {
      spr.setTextColor(SCREEN_FG, SCREEN_BG);
    }
    spr.setCursor(6, y);
    spr.print(items[i]);
    y += ROW_HEIGHT;
  }
  drawFooter("5-way: navigate  A: select  C: exit");
  spr.pushSprite(0, 0);
}

// ------------ Logging ------------

void logAllScalesAveraged() {
  if (!ensureSD()) return;
  ensureLogHeader();

  const uint32_t timestamp = millis();

  File f = SD.open(LOG_FILE, FILE_WRITE);
  if (!f) return;

  for (uint8_t i = 0; i < NUM_SCALES; i++) {
    // Render progress
    drawHeader("Logging...");
    clearContent();
    spr.setTextSize(TEXT_SIZE);
    spr.setCursor(6, HEADER_HEIGHT + 8);
    spr.printf("Scale %u of %u", i + 1, NUM_SCALES);
    spr.setCursor(6, HEADER_HEIGHT + 8 + ROW_HEIGHT);
    spr.print("Averaging 100 samples ~10s");
    drawFooter("C: cancel logging");
    spr.pushSprite(0, 0);

    float avg = averageWeightOverWindow(i);

    // Write CSV line: timestamp, index, avg
    f.print(timestamp);
    f.print(",");
    f.print(i);
    f.print(",");
    f.println(avg, 4);

    if (btnC.isPressed()) {
      break;
    }
  }

  f.close();
}

// ------------ Setup and loop ------------

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(3);
  spr.createSprite(tft.width(), tft.height());
  spr.setColorDepth(16);
  spr.fillSprite(SCREEN_BG);
  spr.setTextColor(SCREEN_FG, SCREEN_BG);
  spr.setTextSize(TEXT_SIZE);

  // Inputs
  btnA.begin(WIO_KEY_A);
  btnB.begin(WIO_KEY_B);
  btnC.begin(WIO_KEY_C);
  joyUp.begin(WIO_5S_UP);
  joyDown.begin(WIO_5S_DOWN);
  joyLeft.begin(WIO_5S_LEFT);
  joyRight.begin(WIO_5S_RIGHT);
  joyPress.begin(WIO_5S_PRESS);

  // SD and calibration
  sdOk = SD.begin(SDCARD_SS_PIN);
  if (!sdOk) {
    drawHeader("SD Error");
    clearContent();
    spr.setCursor(6, HEADER_HEIGHT + 8);
    spr.setTextColor(SCREEN_ERR, SCREEN_BG);
    spr.print("SD init failed!");
    spr.pushSprite(0, 0);
    delay(1500);
  }
  loadCalibration();
  ensureLogHeader();

  // Scales
  initScales();

  // Start BLE service and notifier task
  bleInit();

  lastLogTimeMs = millis();

  currentMode = MODE_MEASURE;
  selectedIndex = 0;
  listPage = 0;
  renderMeasure();
}

void handleNavigation(uint8_t totalItems) {
  // Up/Down scroll; Left/Right page when many
  if (evDown == 1) {
    if (selectedIndex + 1 < totalItems) {
      selectedIndex++;
      uint8_t start = listPage * itemsPerPage;
      if (selectedIndex >= start + itemsPerPage) {
        listPage++;
      }
    }
  }
  if (evUp == 1) {
    if (selectedIndex > 0) {
      selectedIndex--;
      uint8_t start = listPage * itemsPerPage;
      if (selectedIndex < start) {
        if (listPage > 0) listPage--;
      }
    }
  }
  if (evRight == 1) {
    uint8_t maxPage = (totalItems == 0) ? 0 : (uint8_t)((totalItems - 1) / itemsPerPage);
    if (listPage < maxPage) {
      listPage++;
      selectedIndex = min<uint8_t>(selectedIndex + itemsPerPage, totalItems - 1);
    }
  }
  if (evLeft == 1) {
    if (listPage > 0) {
      listPage--;
      selectedIndex = (selectedIndex >= itemsPerPage) ? (selectedIndex - itemsPerPage) : 0;
    }
  }
}

void tareScale(uint8_t idx) {
  drawHeader("Tare...");
  clearContent();
  tft.setCursor(6, HEADER_HEIGHT + 8);
  tft.print("Taring scale ");
  tft.print(idx);
  tft.print("...");
  scales[idx].tare();
  tareOffsets[idx] = scales[idx].get_offset();
  saveCalibration(); // persist new offset
  delay(400);
}

void loop() {
  // Update button states once and cache events
  evBtnA = btnA.update();
  evBtnB = btnB.update();
  evBtnC = btnC.update();
  evUp = joyUp.update();
  evDown = joyDown.update();
  evLeft = joyLeft.update();
  evRight = joyRight.update();
  evPress = joyPress.update();

  // Periodic logging (non-blocking trigger; logging itself averages with delays)
  if ((uint32_t)(millis() - lastLogTimeMs) >= LOG_INTERVAL_MS) {
    logAllScalesAveraged();
    lastLogTimeMs = millis();
    renderMeasure();
  }

  // Non-blocking live smoothing to keep buttons responsive
  for (uint8_t i = 0; i < NUM_SCALES; i++) {
    float sample = scales[i].get_units(1);
    weightsEma[i] = (1.0f - UI_EMA_ALPHA) * weightsEma[i] + UI_EMA_ALPHA * sample;
  }

  // Push latest weights to BLE at 5 Hz via RTOS notify task
  bleUpdateWeights(weightsEma, NUM_SCALES);

  // Mode handling
  switch (currentMode) {
    case MODE_MEASURE: {
      // Interactions
      handleNavigation(NUM_SCALES);

      // A: jump to calibrate selected
      if (evBtnA == 1) {
        currentMode = MODE_CALIBRATE;
        listPage = selectedIndex / max<uint8_t>(1, itemsPerPage);
        renderCalibrateList();
        break;
      }
      // B: tare selected scale
      if (evBtnB == 1) {
        tareScale(selectedIndex);
        renderMeasure();
        break;
      }
      // C: open menu
      if (evBtnC == 1 || evPress == 1) {
        currentMode = MODE_SETTINGS;
        renderMenu();
        break;
      }

      // Refresh weights view periodically
      static uint32_t lastUI = 0;
      if ((uint32_t)(millis() - lastUI) >= 500) {
        renderMeasure();
        lastUI = millis();
      }
    } break;

    case MODE_CALIBRATE: {
      handleNavigation(NUM_SCALES);

      // B: tare selected
      if (evBtnB == 1) {
        tareScale(selectedIndex);
        renderCalibrateList();
        break;
      }
      // A: guided two-point calibration (0g then 100g)
      if (evBtnA == 1) {
        // Step 1: ensure empty, confirm to tare (smooth live preview)
        while (true) {
          float sample = scales[selectedIndex].get_units(1);
          weightsEma[selectedIndex] = (1.0f - UI_EMA_ALPHA) * weightsEma[selectedIndex] + UI_EMA_ALPHA * sample;
          float preview = weightsEma[selectedIndex];
          renderCalibrateEditStep(selectedIndex, "Calib: Zero", "Remove weight, press A to tare", preview);
          int8_t a = btnA.update();
          int8_t b = btnB.update();
          int8_t c = btnC.update();
          if (b == 1) { tareScale(selectedIndex); }
          if (a == 1) {
            scales[selectedIndex].tare();
            tareOffsets[selectedIndex] = scales[selectedIndex].get_offset();
            break;
          }
          if (c == 1) { renderCalibrateList(); goto calib_exit; }
          delay(10);
        }
        // Step 2: place 100g, confirm to capture 10s average
        while (true) {
          float sample = scales[selectedIndex].get_units(1);
          weightsEma[selectedIndex] = (1.0f - UI_EMA_ALPHA) * weightsEma[selectedIndex] + UI_EMA_ALPHA * sample;
          float preview = weightsEma[selectedIndex];
          renderCalibrateEditStep(selectedIndex, "Calib: 100g", "Place 100g, press A to capture", preview);
          int8_t a = btnA.update();
          int8_t b = btnB.update();
          int8_t c = btnC.update();
          if (b == 1) { tareScale(selectedIndex); }
          if (a == 1) {
            long long sum = 0;
            for (uint16_t i = 0; i < AVG_SAMPLES; i++) {
              sum += (long)scales[selectedIndex].get_value(1);
              delay(AVG_SAMPLE_DELAY_MS);
            }
            float avgValue = (float)sum / (float)AVG_SAMPLES; // raw units
            float scale = avgValue / 100.0f;
            //if (scale < 0) scale = -scale; // ensure positive slope
            calibrationFactors[selectedIndex] = scale;
            scales[selectedIndex].set_scale(scale);
            // Keep the zero point from step 1 (do not re-tare here)
            saveCalibration();
            renderCalibrateList();
            break;
          }
          if (c == 1) { renderCalibrateList(); break; }
          delay(10);
        }
        calib_exit:
        break;
      }
      // C: back to measure
      if (evBtnC == 1 || evPress == 1) {
        currentMode = MODE_MEASURE;
        renderMeasure();
        break;
      }

      // Refresh list view periodically
      static uint32_t lastUI = 0;
      if ((uint32_t)(millis() - lastUI) >= 500) {
        renderCalibrateList();
        lastUI = millis();
      }
    } break;

    case MODE_SETTINGS: {
      // Navigate between menu entries using up/down
      if (evDown == 1) {
        currentMode = (AppMode)(((int)currentMode + 1) % 3);
        renderMenu();
      }
      if (evUp == 1) {
        currentMode = (AppMode)(((int)currentMode - 1 + 3) % 3);
        renderMenu();
      }
      if (evBtnA == 1) {
        // Enter selected mode
        if (currentMode == MODE_MEASURE) {
          renderMeasure();
        } else if (currentMode == MODE_CALIBRATE) {
          listPage = selectedIndex / max<uint8_t>(1, itemsPerPage);
          renderCalibrateList();
        } else {
          // Settings page placeholder
          drawHeader("Settings");
          clearContent();
          spr.setTextSize(TEXT_SIZE);
          spr.setCursor(6, HEADER_HEIGHT + 8);
          spr.print("SD: ");
          spr.print(sdOk ? "OK" : "Error");
          spr.setCursor(6, HEADER_HEIGHT + 8 + ROW_HEIGHT);
          spr.printf("Scales: %u", NUM_SCALES);
          drawFooter("C: back");
          spr.pushSprite(0, 0);
        }
      }
      if (evBtnC == 1 || evPress == 1) {
        currentMode = MODE_MEASURE;
        renderMeasure();
      }
    } break;
  }

  delay(10);
}
