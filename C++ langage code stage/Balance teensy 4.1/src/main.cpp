// ============================================================
// Teensy 4.1 — 24 balances HX711
// SCK commun pin 2 | DOUT pins 3-26
// UART vers reTerminal : Serial1 (TX pin 1) à 115200 baud
//
// Format trame envoyée toutes les SEND_INTERVAL_MS :
// B1:228,50;B2:ERR;B3:145,30;...;B24:ERR\n
//
// Calibration stockée en EEPROM — survit aux reboots
// Balance absente → marqueur ERR dans la trame
// ============================================================

#include <Arduino.h>
#include <HX711.h>
#include <EEPROM.h>

// ============================================================
// CONFIG
// ============================================================

#define NB_SCALES           24
#define COMMON_SCK_PIN       2

// DOUT pins : B1=pin3, B2=pin4, ..., B24=pin27
const int DOUT_PINS[NB_SCALES] = {
     3,  4,  5,  6,  7,  8,  9, 10, 11, 12,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27
};

#define NB_SAMPLES          5
#define TARE_SAMPLES        40
#define CALIB_SAMPLES       80
#define CALIBRATION_MASS    100.0f
#define ZERO_DEADBAND_G     0.05f
#define SEND_INTERVAL_MS    5000

// ============================================================
// EEPROM
// ============================================================

#define EEPROM_BASE_TARE    0
#define EEPROM_BASE_FACTOR  (NB_SCALES * sizeof(float))

void eeprom_save_tare(int i, float v)   { EEPROM.put(EEPROM_BASE_TARE   + i * sizeof(float), v); }
void eeprom_save_factor(int i, float v) { EEPROM.put(EEPROM_BASE_FACTOR + i * sizeof(float), v); }

float eeprom_load_tare(int i) {
    float v; EEPROM.get(EEPROM_BASE_TARE + i * sizeof(float), v);
    return (isnan(v) || isinf(v)) ? 0.0f : v;
}
float eeprom_load_factor(int i) {
    float v; EEPROM.get(EEPROM_BASE_FACTOR + i * sizeof(float), v);
    return (isnan(v) || isinf(v) || v == 0.0f) ? 1.0f : v;
}

// ============================================================
// VARIABLES GLOBALES
// ============================================================

HX711    scales[NB_SCALES];
bool     scale_present[NB_SCALES];
float    tare_offsets[NB_SCALES];
float    calibration_factors[NB_SCALES];
float    last_weights[NB_SCALES];
long     last_raws[NB_SCALES];
uint32_t last_send_ms    = 0;
uint32_t send_interval_ms = SEND_INTERVAL_MS;
int      read_index      = 0;

// ============================================================
// DETECTION HX711 — simple et fiable
//
// Principe : un HX711 connecté met DOUT à LOW quand il a
// une donnée prête (toutes les ~100 ms à 10 Hz).
// Avec INPUT_PULLUP, un pin non connecté reste HIGH.
// On attend juste un LOW franc pendant 1200 ms max.
// Ensuite on tente une vraie lecture pour confirmer.
// ============================================================

bool hx711_detect(int index) {
    int dout = DOUT_PINS[index];

    // SCK LOW = mode normal (pas de power-down)
    pinMode(COMMON_SCK_PIN, OUTPUT);
    digitalWrite(COMMON_SCK_PIN, LOW);

    // DOUT en entrée avec pull-up : pin flottant → reste HIGH
    pinMode(dout, INPUT_PULLUP);
    delay(10);

    // Étape 1 : attendre DOUT LOW (HX711 données prêtes)
    // Si rien au bout de 1200 ms → absent
    unsigned long t = millis();
    bool got_low = false;
    while (millis() - t < 1200) {
        if (digitalRead(dout) == LOW) { got_low = true; break; }
        delay(10);
    }
    if (!got_low) return false;

    // Étape 2 : initialiser et tenter une lecture réelle
    // Si le HX711 est là, read() renvoie une valeur 24 bits non nulle
    scales[index].begin(dout, COMMON_SCK_PIN);
    delay(10);

    if (!scales[index].is_ready()) return false;

    long val = scales[index].read();

    // Une valeur brute de 0 exact ou très proche de 0x7FFFFF (saturé)
    // indique un problème — on accepte tout le reste
    if (val == 0 || val == 0x7FFFFF || val == -0x800000) return false;

    return true;
}

// ============================================================
// LECTURE RAW
// ============================================================

long read_raw_average(int index, int n) {
    if (!scale_present[index]) return last_raws[index];
    long total = 0; int valid = 0;
    for (int i = 0; i < n; i++) {
        if (scales[index].is_ready()) { total += scales[index].read(); valid++; }
        delay(3);
    }
    return (valid == 0) ? last_raws[index] : total / valid;
}

long read_raw_exact(int index, int n) {
    if (!scale_present[index]) return 0;
    long total = 0; int valid = 0;
    while (valid < n) {
        unsigned long w = millis();
        while (!scales[index].is_ready()) {
            if (millis() - w > 1500) return (valid > 0) ? total / valid : last_raws[index];
            delay(1);
        }
        total += scales[index].read();
        valid++;
        delay(1);
    }
    return total / valid;
}

// ============================================================
// POIDS
// ============================================================

float raw_to_weight(int index, long raw) {
    float factor = calibration_factors[index];
    if (factor == 0.0f) return 0.0f;
    float w = (float)(raw - (long)tare_offsets[index]) / factor;
    if (fabsf(w) < ZERO_DEADBAND_G) w = 0.0f;
    return w;
}

void read_weight(int index) {
    if (!scale_present[index]) { last_weights[index] = -1.0f; last_raws[index] = 0; return; }
    long raw = read_raw_average(index, NB_SAMPLES);
    last_raws[index]    = raw;
    last_weights[index] = raw_to_weight(index, raw);
}

// ============================================================
// TARE
// ============================================================

void tare(int index) {
    if (!scale_present[index]) { Serial1.printf("# TARE B%d : absente\n", index+1); return; }
    Serial1.printf("# TARE B%d : mesure en cours...\n", index+1);
    delay(500);
    long raw = read_raw_exact(index, TARE_SAMPLES);
    tare_offsets[index] = (float)raw;
    last_raws[index]    = raw;
    last_weights[index] = 0.0f;
    eeprom_save_tare(index, tare_offsets[index]);
    Serial1.printf("# TARE B%d : OK (raw=%ld)\n", index+1, raw);
}

// ============================================================
// CALIBRATION
// ============================================================

void calibrate(int index) {
    if (!scale_present[index]) { Serial1.printf("# CALIB B%d : absente\n", index+1); return; }

    Serial1.printf("# CALIB B%d : retirez tout, puis envoyez OK\n", index+1);
    while (true) {
        if (Serial1.available()) { String c = Serial1.readStringUntil('\n'); c.trim(); if (c == "OK") break; }
        delay(50);
    }
    delay(500);
    long raw_0g = read_raw_exact(index, CALIB_SAMPLES);
    tare_offsets[index] = (float)raw_0g;
    last_weights[index] = 0.0f;
    eeprom_save_tare(index, tare_offsets[index]);
    Serial1.printf("# CALIB B%d : 0g OK (raw=%ld) — posez %.0fg, puis envoyez OK\n", index+1, raw_0g, CALIBRATION_MASS);

    while (true) {
        if (Serial1.available()) { String c = Serial1.readStringUntil('\n'); c.trim(); if (c == "OK") break; }
        delay(50);
    }
    delay(500);
    long raw_ref = read_raw_exact(index, CALIB_SAMPLES);
    long diff = raw_ref - raw_0g;
    if (abs(diff) < 10) { Serial1.printf("# CALIB B%d : ERREUR diff=%ld\n", index+1, diff); return; }
    calibration_factors[index] = (float)diff / CALIBRATION_MASS;
    eeprom_save_factor(index, calibration_factors[index]);
    Serial1.printf("# CALIB B%d : OK (facteur=%.4f)\n", index+1, calibration_factors[index]);
}

// ============================================================
// TRAME UART
// ============================================================

void send_uart_frame() {
    for (int i = 0; i < NB_SCALES; i++) {
        Serial1.printf("B%d:", i+1);
        if (!scale_present[i] || last_weights[i] < -0.5f) {
            Serial1.print("ERR");
        } else {
            char buf[12];
            dtostrf(last_weights[i], 6, 2, buf);
            for (int j = 0; buf[j]; j++) if (buf[j] == '.') buf[j] = ',';
            Serial1.print(buf);
        }
        if (i < NB_SCALES - 1) Serial1.print(";");
    }
    Serial1.print("\n");
}

// ============================================================
// COMMANDES ENTRANTES
// ============================================================

void handle_serial_command(String cmd) {
    cmd.trim();
    if (cmd.startsWith("TARE:")) {
        int idx = cmd.substring(5).toInt() - 1;
        if (idx >= 0 && idx < NB_SCALES) tare(idx);
        else Serial1.printf("# TARE : index invalide\n");

    } else if (cmd.startsWith("CALIB:")) {
        int idx = cmd.substring(6).toInt() - 1;
        if (idx >= 0 && idx < NB_SCALES) calibrate(idx);
        else Serial1.printf("# CALIB : index invalide\n");

    } else if (cmd == "STATUS") {
        Serial1.println("# STATUS :");
        for (int i = 0; i < NB_SCALES; i++)
            Serial1.printf("# B%02d : %s | facteur=%.4f | tare=%.0f\n",
                i+1, scale_present[i] ? "PRESENTE" : "ABSENTE",
                calibration_factors[i], tare_offsets[i]);

    } else if (cmd.startsWith("INTERVAL:")) {
        uint32_t val = cmd.substring(9).toInt();
        if (val >= 1000) { send_interval_ms = val; Serial1.printf("# Intervalle : %lu ms\n", send_interval_ms); }
        else Serial1.println("# INTERVAL : minimum 1000 ms");

    } else if (cmd != "OK") {
        Serial1.printf("# Commande inconnue : %s\n", cmd.c_str());
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    Serial.println("=== Teensy 4.1 — Balances Cosmo ===");
    Serial1.println("# Teensy 4.1 — Balances Cosmo — demarrage");

    // SCK LOW dès le départ pour que les HX711 ne soient pas en power-down
    pinMode(COMMON_SCK_PIN, OUTPUT);
    digitalWrite(COMMON_SCK_PIN, LOW);
    delay(100);

    // Charge calibration EEPROM
    for (int i = 0; i < NB_SCALES; i++) {
        tare_offsets[i]        = eeprom_load_tare(i);
        calibration_factors[i] = eeprom_load_factor(i);
        last_weights[i]        = 0.0f;
        last_raws[i]           = 0;
        scale_present[i]       = false;
    }

    // Détection — on teste chaque pin dans l'ordre
    Serial.println("--- Detection des balances ---");
    Serial1.println("# Detection des balances...");

    for (int i = 0; i < NB_SCALES; i++) {
        Serial.printf("Test B%02d (pin %d)... ", i+1, DOUT_PINS[i]);
        if (hx711_detect(i)) {
            scale_present[i] = true;
            Serial.println("DETECTEE");
            Serial1.printf("# B%02d : detectee\n", i+1);
        } else {
            // Remettre le pin en INPUT_PULLUP propre si absent
            pinMode(DOUT_PINS[i], INPUT_PULLUP);
            scale_present[i] = false;
            Serial.println("absente");
            Serial1.printf("# B%02d : absente\n", i+1);
        }
        // Petite pause entre chaque détection pour laisser le bus se stabiliser
        delay(50);
    }

    // Résumé
    int nb_present = 0;
    for (int i = 0; i < NB_SCALES; i++) if (scale_present[i]) nb_present++;
    Serial.printf("--- %d balance(s) detectee(s) sur %d ---\n", nb_present, NB_SCALES);
    Serial1.printf("# %d balance(s) detectee(s) — envoi toutes les %lu ms\n", nb_present, send_interval_ms);
}

// ============================================================
// LOOP
// ============================================================

void loop() {
    // 1. Lecture tournante — une balance par cycle
    read_weight(read_index);
    read_index = (read_index + 1) % NB_SCALES;

    // 2. Commandes entrantes reTerminal
    if (Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        handle_serial_command(cmd);
    }

    // 3. Envoi trame UART
    if (millis() - last_send_ms >= send_interval_ms) {
        send_uart_frame();
        last_send_ms = millis();
    }

    delay(5);
}