// ============================================================
// Wio Terminal + HX711 (2 à 4 balances)
// Version V3.0 — SANS WIFI
// Communication via USB Serial vers reTerminal (Raspberry Pi)
//
// PROTOCOLE SERIAL :
//   Le Wio envoie toutes les 6h une ligne :
//   DATA,<poids_b1>,<poids_b2>,<poids_b3>,<poids_b4>
//   Exemple : DATA,48.72,32.15,0.00,0.00
//   0.00 = balance absente ou non connectée
//
//   Ce format est identique à ce que le Teensy enverra plus tard.
//   Le reTerminal n'aura pas besoin d'être modifié lors du remplacement.
//
// BOUTONS :
//   A        → Tare de la balance active
//   B        → Calibration de la balance active
//   C        → Basculer OVERVIEW / FOCUS
//   Gauche   → Balance active précédente
//   Droite   → Balance active suivante
//   Bas      → Envoyer les données maintenant (test)
//
// CHANGER L'INTERVALLE :
//   Ligne #define RECORD_INTERVAL_MS
//   6h  → 6UL  * 3600UL * 1000UL  (test)
//   24h → 24UL * 3600UL * 1000UL  (production)
// ============================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <HX711.h>
#include <stdarg.h>

// ============================================================
// CONFIG MATÉRIELLE
// ============================================================

#define NB_SCALES         4     // Nombre max de balances (changer si besoin)
#define CALIBRATION_MASS  100.0f

#define NB_SAMPLES        5
#define TARE_SAMPLES      40
#define CALIB_SAMPLES     80

#define DISPLAY_PERIOD_MS 300
#define DEBOUNCE_MS       150

#define ZERO_DEADBAND_G     0.05f
#define DISPLAY_DEADBAND_G  0.03f

const int COMMON_SCK_PIN         = D0;
const int DOUT_PINS[NB_SCALES]   = { D1, D2, D3, D4 };

// ============================================================
// CONFIG INTERVALLE
// 6h pour les tests, passer à 24h pour la production
// ============================================================

#define RECORD_INTERVAL_MS  (6UL * 3600UL * 1000UL)
// #define RECORD_INTERVAL_MS  (24UL * 3600UL * 1000UL)

// ============================================================
// VARIABLES GLOBALES
// ============================================================

float tare_offsets[NB_SCALES]        = {0.0f, 0.0f, 0.0f, 0.0f};
float calibration_factors[NB_SCALES] = {1.0f, 1.0f, 1.0f, 1.0f};

int  active_scale        = 0;
bool mode_focus          = false;
int  overview_read_index = 0;

float last_weights[NB_SCALES];
long  last_raws[NB_SCALES];
float displayed_weights[NB_SCALES];
bool  scale_present[NB_SCALES];

float last_drawn_weight[NB_SCALES] = {999999, 999999, 999999, 999999};
long  last_drawn_raw[NB_SCALES]    = {-999999, -999999, -999999, -999999};
int   last_drawn_active_scale      = -1;
bool  last_drawn_mode_focus        = false;

uint32_t last_record_ms = 0;

HX711    scales[NB_SCALES];
TFT_eSPI tft = TFT_eSPI();

// ============================================================
// COULEURS
// ============================================================

#define COLOR_TITLE    0x5D7F
#define COLOR_MODE     0xFFE0
#define COLOR_WHITE    TFT_WHITE
#define COLOR_GRAY     0x8410
#define COLOR_GREEN    0x07E0
#define COLOR_ORANGE   0xFD20
#define COLOR_STATUS   0x07E0
#define COLOR_RAW      0xAD55

// ============================================================
// POSITIONS ÉCRAN
// ============================================================

const int Y_TITLE  = 15;
const int Y_MODE   = 45;
const int Y_LINES[NB_SCALES] = { 75, 105, 135, 165 };
const int Y_BIG    = 95;
const int Y_RAW    = 165;
const int Y_CALIB  = 185;
const int Y_STATUS = 220;

char status_text[64] = "Demarrage...";

// ============================================================
// PROTOTYPES
// ============================================================

void draw_status_bar();
void draw_static_screen();
void update_display_values();
void force_full_redraw();
void send_data_serial();

// ============================================================
// STATUS BAR
// ============================================================

void set_statusf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(status_text, sizeof(status_text), fmt, args);
    va_end(args);
    Serial.flush(); // Ne pas interrompre un envoi de données
    draw_status_bar();
}

// ============================================================
// BOUTONS
// ============================================================

struct ButtonEdge {
    int      pin;
    bool     last_state;
    uint32_t last_event_time_ms;

    void begin(int p) {
        pin = p;
        pinMode(pin, INPUT_PULLUP);
        last_state = (digitalRead(pin) == LOW);
        last_event_time_ms = 0;
    }

    bool fell() {
        uint32_t now = millis();
        bool current = (digitalRead(pin) == LOW);
        bool event = false;
        if (current && !last_state) {
            if (now - last_event_time_ms >= DEBOUNCE_MS) {
                event = true;
                last_event_time_ms = now;
            }
        }
        last_state = current;
        return event;
    }

    bool is_pressed() {
        return (digitalRead(pin) == LOW);
    }
};

ButtonEdge btn_a, btn_b, btn_c, btn_left, btn_right, btn_down;

// ============================================================
// DÉTECTION HX711
// ============================================================

bool hx711_detect_dout(int dout_pin, unsigned long timeout_ms = 1000) {
    pinMode(dout_pin, INPUT);
    pinMode(COMMON_SCK_PIN, OUTPUT);
    digitalWrite(COMMON_SCK_PIN, LOW);

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        if (digitalRead(dout_pin) == LOW) return true;
        delay(10);
    }
    return false;
}

// ============================================================
// MESURES
// ============================================================

long read_raw_average(int index, int n = NB_SAMPLES) {
    if (index < 0 || index >= NB_SCALES) return 0;
    if (!scale_present[index]) return last_raws[index];

    long total = 0;
    int valid = 0;

    for (int i = 0; i < n; i++) {
        if (scales[index].is_ready()) {
            total += scales[index].read();
            valid++;
        }
        delay(3);
    }

    return (valid == 0) ? last_raws[index] : total / valid;
}

long read_raw_average_exact(int index, int n, const char* label) {
    if (index < 0 || index >= NB_SCALES) return 0;
    if (!scale_present[index]) return 0;

    long total = 0;
    int valid = 0;

    while (valid < n) {
        unsigned long wait_start = millis();
        while (!scales[index].is_ready()) {
            if (millis() - wait_start > 1200) {
                if (valid > 0) { set_statusf("%s timeout %d/%d", label, valid, n); return total / valid; }
                else           { set_statusf("%s timeout 0/%d", label, n); return last_raws[index]; }
            }
            delay(1);
        }
        total += scales[index].read();
        valid++;
        if (valid % 10 == 0 || valid == n) set_statusf("%s %d/%d", label, valid, n);
        delay(1);
    }
    return total / valid;
}

float raw_to_weight(int index, long raw) {
    if (index < 0 || index >= NB_SCALES) return 0.0f;
    float factor = calibration_factors[index];
    if (factor == 0.0f) return 0.0f;
    float weight = (float)(raw - (long)tare_offsets[index]) / factor;
    if (abs(weight) < ZERO_DEADBAND_G) weight = 0.0f;
    return weight;
}

void read_weight(int index) {
    if (index < 0 || index >= NB_SCALES) return;
    if (!scale_present[index]) {
        last_weights[index] = 0.0f;
        last_raws[index] = 0;
        return;
    }
    long raw = read_raw_average(index, NB_SAMPLES);
    last_raws[index] = raw;
    last_weights[index] = raw_to_weight(index, raw);
}

float stable_display_weight(int index) {
    if (index < 0 || index >= NB_SCALES) return 0.0f;
    float weight = last_weights[index];
    float old = displayed_weights[index];
    if (abs(weight - old) >= DISPLAY_DEADBAND_G) displayed_weights[index] = weight;
    return displayed_weights[index];
}

// ============================================================
// TARE / CALIBRATION
// ============================================================

void tare(int index) {
    if (index < 0 || index >= NB_SCALES) return;
    if (!scale_present[index]) { set_statusf("B%d absente", index + 1); return; }

    set_statusf("Tare B%d: ne touche plus", index + 1);
    delay(1200);
    long raw = read_raw_average_exact(index, TARE_SAMPLES, "Tare");
    last_raws[index] = raw;
    tare_offsets[index] = (float)raw;
    last_weights[index] = 0.0f;
    displayed_weights[index] = 0.0f;
    last_drawn_weight[index] = 999999;
    last_drawn_raw[index] = -999999;
    set_statusf("Tare B%d OK", index + 1);
    update_display_values();
    delay(800);
}

void wait_release_b() { while (btn_b.is_pressed()) delay(20); delay(DEBOUNCE_MS); }
void wait_press_b()   { while (!btn_b.is_pressed()) delay(20); delay(DEBOUNCE_MS); }
void wait_b_click()   { wait_release_b(); wait_press_b(); wait_release_b(); }

void calibrate_0_100g(int index) {
    if (index < 0 || index >= NB_SCALES) return;
    if (!scale_present[index]) { set_statusf("B%d absente", index + 1); return; }

    set_statusf("Calib B%d: vide puis B", index + 1);
    wait_b_click();
    set_statusf("0g: ne touche plus");
    delay(1500);
    long raw_0g = read_raw_average_exact(index, CALIB_SAMPLES, "Mesure 0g");
    last_raws[index] = raw_0g;
    tare_offsets[index] = (float)raw_0g;
    displayed_weights[index] = 0.0f;
    last_weights[index] = 0.0f;
    last_drawn_weight[index] = 999999;
    last_drawn_raw[index] = -999999;
    set_statusf("0g OK");
    update_display_values();
    delay(1000);

    set_statusf("Pose %.0fg puis B", CALIBRATION_MASS);
    wait_b_click();
    set_statusf("%.0fg: ne touche plus", CALIBRATION_MASS);
    delay(1500);
    long raw_ref = read_raw_average_exact(index, CALIB_SAMPLES, "Mesure masse");
    last_raws[index] = raw_ref;
    long diff = raw_ref - raw_0g;

    if (abs(diff) < 10) { set_statusf("Erreur calib diff=%ld", diff); return; }

    calibration_factors[index] = (float)diff / CALIBRATION_MASS;
    last_weights[index] = CALIBRATION_MASS;
    displayed_weights[index] = CALIBRATION_MASS;
    last_drawn_weight[index] = 999999;
    last_drawn_raw[index] = -999999;
    set_statusf("Calib B%d OK f=%.2f", index + 1, calibration_factors[index]);
    update_display_values();
    delay(1200);
}

// ============================================================
// NAVIGATION
// ============================================================

void change_active_scale(int direction) {
    active_scale = (active_scale + direction + NB_SCALES) % NB_SCALES;
    snprintf(status_text, sizeof(status_text), "Balance active: B%d", active_scale + 1);
}

// ============================================================
// ENVOI DONNÉES VIA SERIAL (USB vers reTerminal)
// ============================================================

void send_data_serial() {
    // Lire toutes les balances une dernière fois avant d'envoyer
    for (int i = 0; i < NB_SCALES; i++) {
        read_weight(i);
    }

    // Format : DATA,<b1>,<b2>,<b3>,<b4>
    // 0.00 si la balance est absente
    Serial.print("DATA");
    for (int i = 0; i < NB_SCALES; i++) {
        Serial.print(",");
        if (scale_present[i]) {
            Serial.print(stable_display_weight(i), 2);
        } else {
            Serial.print("0.00");
        }
    }
    Serial.println(); // \n final — le reTerminal lit jusqu'au \n

    // Affichage sur l'écran
    unsigned long h = (millis() / 1000) / 3600;
    unsigned long m = ((millis() / 1000) % 3600) / 60;
    set_statusf("Envoye T+%02luh%02lu | BAS=forcer", h, m);
}

// ============================================================
// AFFICHAGE
// ============================================================

void reset_draw_cache() {
    for (int i = 0; i < NB_SCALES; i++) {
        last_drawn_weight[i] = 999999;
        last_drawn_raw[i]    = -999999;
    }
    last_drawn_active_scale = active_scale;
    last_drawn_mode_focus   = mode_focus;
}

void draw_status_bar() {
    tft.fillRect(0, Y_STATUS, 320, 20, TFT_BLACK);
    tft.setTextColor(COLOR_STATUS, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, Y_STATUS);
    tft.print(status_text);
}

void draw_static_screen() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(COLOR_TITLE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, Y_TITLE);

    if (mode_focus) {
        tft.printf("Balance B%d", active_scale + 1);
        tft.setTextColor(COLOR_MODE, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, Y_MODE);
        tft.print("FOCUS | A tare | B calib | C overview");
    } else {
        tft.print("Balances HX711");
        tft.setTextColor(COLOR_MODE, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, Y_MODE);
        tft.print("OVERVIEW | A tare | B calib | C focus");
    }

    // Indicateur de prochaine mesure
    tft.setTextColor(COLOR_ORANGE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, Y_MODE + 15);

    unsigned long next_ms = RECORD_INTERVAL_MS - (millis() - last_record_ms);
    unsigned long next_h  = (next_ms / 1000) / 3600;
    unsigned long next_m  = ((next_ms / 1000) % 3600) / 60;
    tft.printf("Prochain envoi dans %02luh%02lu | BAS=forcer", next_h, next_m);

    if (mode_focus) {
        tft.setTextColor(COLOR_RAW, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, Y_RAW);
        tft.print("RAW:");
        tft.setCursor(10, Y_CALIB);
        tft.print("Calib:");
    }

    draw_status_bar();
}

void force_full_redraw() {
    draw_static_screen();
    reset_draw_cache();
    update_display_values();
}

void update_display_values() {
    if (mode_focus) {
        if (!scale_present[active_scale]) {
            tft.setTextColor(COLOR_WHITE, TFT_BLACK);
            tft.setTextSize(4);
            tft.setCursor(10, Y_BIG);
            tft.print("Absente    ");
            return;
        }

        float shown = stable_display_weight(active_scale);
        long  raw   = last_raws[active_scale];

        if (abs(shown - last_drawn_weight[active_scale]) >= DISPLAY_DEADBAND_G) {
            tft.setTextColor(COLOR_WHITE, TFT_BLACK);
            tft.setTextSize(4);
            tft.setCursor(10, Y_BIG);
            tft.printf("%10.2f g", shown);
            last_drawn_weight[active_scale] = shown;
        }

        if (raw != last_drawn_raw[active_scale]) {
            tft.setTextColor(COLOR_RAW, TFT_BLACK);
            tft.setTextSize(1);
            tft.setCursor(50, Y_RAW);
            tft.printf("%-16ld", raw);
            tft.setCursor(60, Y_CALIB);
            tft.printf("%-12.2f", calibration_factors[active_scale]);
            last_drawn_raw[active_scale] = raw;
        }
    } else {
        bool active_changed = (last_drawn_active_scale != active_scale);

        for (int i = 0; i < NB_SCALES; i++) {
            float shown = stable_display_weight(i);

            if (active_changed || abs(shown - last_drawn_weight[i]) >= DISPLAY_DEADBAND_G) {
                tft.setTextSize(2);
                tft.setCursor(10, Y_LINES[i]);

                if (!scale_present[i]) {
                    tft.setTextColor(COLOR_GRAY, TFT_BLACK);
                    tft.printf(" B%d: absente       ", i + 1);
                } else {
                    char prefix = (i == active_scale) ? '>' : ' ';
                    tft.setTextColor(i == active_scale ? COLOR_GREEN : COLOR_WHITE, TFT_BLACK);
                    tft.printf("%cB%d: %7.2f g   ", prefix, i + 1, shown);
                }
                last_drawn_weight[i] = shown;
            }
        }
        last_drawn_active_scale = active_scale;
    }
}

// ============================================================
// MISE À JOUR COMPTEUR UNIQUEMENT (sans redessiner l'écran)
// ============================================================

void update_countdown() {
    unsigned long elapsed = millis() - last_record_ms;
    unsigned long remaining_ms = (elapsed >= RECORD_INTERVAL_MS) ? 0 : RECORD_INTERVAL_MS - elapsed;
    unsigned long next_h = (remaining_ms / 1000) / 3600;
    unsigned long next_m = ((remaining_ms / 1000) % 3600) / 60;

    // Efface uniquement la ligne du compteur
    tft.fillRect(0, Y_MODE + 15, 320, 12, TFT_BLACK);
    tft.setTextColor(COLOR_ORANGE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, Y_MODE + 15);
    tft.printf("Prochain envoi dans %02luh%02lu | BAS=forcer", next_h, next_m);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(115200);

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);

    btn_a.begin(WIO_KEY_A);
    btn_b.begin(WIO_KEY_B);
    btn_c.begin(WIO_KEY_C);
    btn_left.begin(WIO_5S_LEFT);
    btn_right.begin(WIO_5S_RIGHT);
    btn_down.begin(WIO_5S_DOWN);

    for (int i = 0; i < NB_SCALES; i++) {
        last_weights[i]      = 0.0f;
        last_raws[i]         = 0;
        displayed_weights[i] = 0.0f;

        if (hx711_detect_dout(DOUT_PINS[i])) {
            scales[i].begin(DOUT_PINS[i], COMMON_SCK_PIN);
            scale_present[i] = true;
        } else {
            scale_present[i] = false;
        }
    }

    force_full_redraw();

    // Première mesure immédiate au démarrage
    send_data_serial();
    last_record_ms = millis();

    set_statusf("Pret - BAS pour forcer envoi");
}

// ============================================================
// LOOP
// ============================================================

uint32_t last_display_ms  = 0;
uint32_t last_countdown_ms = 0;
#define  COUNTDOWN_PERIOD_MS 30000

void loop() {
    // ─── Boutons ────────────────────────────────────────────
    if (btn_a.fell()) tare(active_scale);
    if (btn_b.fell()) calibrate_0_100g(active_scale);

    if (btn_c.fell()) {
        mode_focus = !mode_focus;
        snprintf(status_text, sizeof(status_text),
                 "Mode %s", mode_focus ? "FOCUS" : "OVERVIEW");
        force_full_redraw();
    }

    // Joystick BAS = forcer l'envoi immédiatement (le compteur continue)
    if (btn_down.fell()) {
        send_data_serial();
        // last_record_ms inchangé → le compteur 6h continue normalement
    }

    if (btn_right.fell()) {
        change_active_scale(1);
        if (mode_focus) force_full_redraw();
        else { update_display_values(); draw_status_bar(); }
    }

    if (btn_left.fell()) {
        change_active_scale(-1);
        if (mode_focus) force_full_redraw();
        else { update_display_values(); draw_status_bar(); }
    }

    // ─── Lecture capteurs ────────────────────────────────────
    if (mode_focus) {
        read_weight(active_scale);
    } else {
        read_weight(overview_read_index);
        overview_read_index = (overview_read_index + 1) % NB_SCALES;
    }

    // ─── Affichage périodique ─────────────────────────────────
    if (millis() - last_display_ms >= DISPLAY_PERIOD_MS) {
        update_display_values();
        last_display_ms = millis();
    }

    // ─── Mise à jour compteur toutes les 30s (sans clignotement) ─
    if (millis() - last_countdown_ms >= COUNTDOWN_PERIOD_MS) {
        update_countdown();
        last_countdown_ms = millis();
    }

    // ─── Envoi automatique toutes les 6h ─────────────────────
    if (millis() - last_record_ms >= RECORD_INTERVAL_MS) {
        send_data_serial();
        last_record_ms = millis();
    }

    delay(10);
}
