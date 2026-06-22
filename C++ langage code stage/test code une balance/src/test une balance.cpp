#include <Arduino.h>
#include <TFT_eSPI.h>
#include "HX711.h"

// ============================
// CONFIG WIO TERMINAL
// ============================

// Branchement HX711 sur Wio Terminal
// SCK  -> D0
// DOUT -> D1
const int LOADCELL_DOUT_PIN = D1;
const int LOADCELL_SCK_PIN  = D0;

const float CALIBRATION_WEIGHT_G = 100.0f; // poids de calibration en grammes

HX711 scale;
TFT_eSPI tft = TFT_eSPI();

float calibrationFactor = 1.0f;
float lastDisplayedWeight = 999999.0f;
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_PERIOD_MS = 300;

// Boutons Wio Terminal
const int BTN_TARE  = WIO_KEY_A;
const int BTN_CALIB = WIO_KEY_B;

// Anti-rebond simple
bool lastA = HIGH;
bool lastB = HIGH;
unsigned long lastButtonMs = 0;
const unsigned long DEBOUNCE_MS = 180;

// ============================
// AFFICHAGE
// ============================

void drawStaticScreen() {
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, 15);
    tft.print("HX711 Balance");

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 45);
    tft.print("A=Tare | B=Calib 0g/100g");

    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setCursor(10, 215);
    tft.print("DOUT=D1 | SCK=D0");
}

void showStatus(const char* msg, uint16_t color = TFT_GREEN) {
    tft.fillRect(0, 185, 320, 25, TFT_BLACK);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 192);
    tft.print(msg);

    Serial.println(msg);
}

void showWeight(float weight) {
    if (millis() - lastDisplayMs < DISPLAY_PERIOD_MS) return;
    lastDisplayMs = millis();

    if (abs(weight - lastDisplayedWeight) < 0.05f) return;
    lastDisplayedWeight = weight;

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(4);
    tft.setCursor(20, 95);
    tft.printf("%8.2f g", weight);
}

void clearWeightArea() {
    tft.fillRect(0, 80, 320, 90, TFT_BLACK);
    lastDisplayedWeight = 999999.0f;
}

void showBigText(const char* msg, uint16_t color = TFT_WHITE) {
    clearWeightArea();
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(3);
    tft.setCursor(20, 105);
    tft.print(msg);
}

void showError(const char* msg) {
    showBigText("ERREUR", TFT_RED);
    showStatus(msg, TFT_RED);
}

// ============================
// BOUTONS
// ============================

bool buttonPressed(int pin, bool &lastState) {
    bool current = digitalRead(pin);
    bool pressed = false;

    if (lastState == HIGH && current == LOW) {
        if (millis() - lastButtonMs > DEBOUNCE_MS) {
            pressed = true;
            lastButtonMs = millis();
        }
    }

    lastState = current;
    return pressed;
}

void waitButtonRelease(int pin) {
    while (digitalRead(pin) == LOW) {
        delay(20);
    }
    delay(150);
}

void waitButtonPress(int pin) {
    while (digitalRead(pin) == HIGH) {
        delay(20);
    }
    delay(150);
}

void waitButtonClick(int pin) {
    waitButtonRelease(pin);
    waitButtonPress(pin);
    waitButtonRelease(pin);
}

// ============================
// TARE / CALIBRATION
// ============================

void doTare() {
    if (!scale.is_ready()) {
        showError("HX711 non detecte");
        return;
    }

    showBigText("TARE...", TFT_YELLOW);
    showStatus("Ne touche plus la balance...");
    delay(1000);

    scale.tare(20);

    clearWeightArea();
    showStatus("Tare terminee");
}

void doCalibration100g() {
    if (!scale.is_ready()) {
        showError("HX711 non detecte");
        return;
    }

    // Etape 1 : 0 g
    showBigText("0 g", TFT_YELLOW);
    showStatus("Vide la balance puis appuie sur B");
    waitButtonClick(BTN_CALIB);

    showBigText("MESURE 0g", TFT_YELLOW);
    showStatus("Ne touche plus...");
    delay(1200);

    scale.set_scale(1.0f);
    scale.tare(30);

    // Etape 2 : 100 g
    showBigText("100 g", TFT_YELLOW);
    showStatus("Pose 100g puis appuie sur B");
    waitButtonClick(BTN_CALIB);

    showBigText("MESURE", TFT_YELLOW);
    showStatus("Mesure du poids de reference...");
    delay(1200);

    long rawValue = scale.get_value(30);

    if (abs(rawValue) < 10) {
        showError("Calibration impossible");
        return;
    }

    calibrationFactor = (float)rawValue / CALIBRATION_WEIGHT_G;
    scale.set_scale(calibrationFactor);

    clearWeightArea();
    showStatus("Calibration 100g terminee");

    Serial.print("Facteur calibration = ");
    Serial.println(calibrationFactor);
}

// ============================
// SETUP
// ============================

void setup() {
    Serial.begin(57600);

    pinMode(BTN_TARE, INPUT_PULLUP);
    pinMode(BTN_CALIB, INPUT_PULLUP);

    tft.begin();
    tft.setRotation(3);
    drawStaticScreen();

    showStatus("Initialisation HX711...");

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    if (!scale.is_ready()) {
        showError("HX711 non detecte");
        delay(2000);
    }

    scale.set_scale(calibrationFactor);
    scale.tare(20);

    drawStaticScreen();
    showStatus("Pret | A=tare | B=calib");
}

// ============================
// LOOP
// ============================

void loop() {
    if (buttonPressed(BTN_TARE, lastA)) {
        doTare();
    }

    if (buttonPressed(BTN_CALIB, lastB)) {
        doCalibration100g();
    }

    if (scale.is_ready()) {
        float reading = scale.get_units(10);

        Serial.print("Poids: ");
        Serial.print(reading);
        Serial.println(" g");

        showWeight(reading);
    } else {
        showError("HX711 non detecte");
    }

    delay(100);
}
