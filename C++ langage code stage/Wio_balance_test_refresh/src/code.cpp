// ============================================================
// Wio Terminal + 4 HX711
// Version SCK commun + WiFi HTTP + JSON
// VERSION CORRIGEE ANTI-CRASH FOCUS + AFFICHAGE SANS CLIGNOTEMENT
// + OPTION RETRY WIFI AVEC JOYSTICK BAS
//
// Correction importante :
// - Ancienne version : draw_static_screen() appelait update_display_values()
//   et update_display_values() pouvait rappeler draw_static_screen().
//   Resultat : recursion infinie possible au passage en FOCUS -> crash.
// - Nouvelle version :
//   1) draw_static_screen() dessine seulement le decor statique.
//   2) update_display_values() met seulement les valeurs a jour.
//   3) force_full_redraw() redessine proprement apres changement de mode.
//
// Ajout :
// - Si WiFi ECHEC au demarrage : joystick BAS = retenter connexion WiFi
// ============================================================

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <rpcWiFi.h>
#include <HX711.h>
#include <stdarg.h>

// ============================================================
// CONFIG
// ============================================================

#define NB_SCALES         4
#define CALIBRATION_MASS  100.0f

#define NB_SAMPLES        5
#define TARE_SAMPLES      40
#define CALIB_SAMPLES     80

#define DISPLAY_PERIOD_MS 300
#define DEBOUNCE_MS       150

#define ZERO_DEADBAND_G     0.05f
#define DISPLAY_DEADBAND_G  0.03f

const int COMMON_SCK_PIN = D0;
const int DOUT_PINS[NB_SCALES] = { D1, D2, D3, D4 };

// ============================================================
// WIFI
// ============================================================

const char* WIFI_SSID     = "Leo’s iPhone";
const char* WIFI_PASSWORD = "KaitoLeo06";

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

HX711 scales[NB_SCALES];
TFT_eSPI tft = TFT_eSPI();

// ============================================================
// COULEURS
// ============================================================

#define COLOR_TITLE    0x5D7F
#define COLOR_MODE     0xFFE0
#define COLOR_WHITE    TFT_WHITE
#define COLOR_GRAY     0x8410
#define COLOR_GREEN    0x07E0
#define COLOR_STATUS   0x07E0
#define COLOR_RAW      0xAD55

// ============================================================
// POSITIONS
// ============================================================

const int Y_TITLE  = 15;
const int Y_MODE   = 45;
const int Y_LINES[NB_SCALES] = { 75, 105, 135, 165 };
const int Y_BIG    = 95;
const int Y_RAW    = 165;
const int Y_CALIB  = 185;
const int Y_STATUS = 220;

char status_text[64] = "Demarrage...";

void draw_status_bar();
void draw_static_screen();
void update_display_values();
void force_full_redraw();
void setup_wifi();
void retry_wifi();

void set_statusf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(status_text, sizeof(status_text), fmt, args);
    va_end(args);

    Serial.println(status_text);
    draw_status_bar();
}

// ============================================================
// WIFI SERVER
// ============================================================

WiFiServer server(80);
bool wifi_connected = false;

// ============================================================
// BOUTONS
// ============================================================

struct ButtonEdge {
    int       pin;
    bool      last_state;
    uint32_t  last_event_time_ms;

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
// DETECTION HX711
// ============================================================

bool hx711_detect_dout(int dout_pin, unsigned long timeout_ms = 1000) {
    pinMode(dout_pin, INPUT);
    pinMode(COMMON_SCK_PIN, OUTPUT);
    digitalWrite(COMMON_SCK_PIN, LOW);

    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        if (digitalRead(dout_pin) == LOW) {
            return true;
        }
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

    if (valid == 0) {
        return last_raws[index];
    }

    return total / valid;
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
                if (valid > 0) {
                    set_statusf("%s timeout %d/%d", label, valid, n);
                    return total / valid;
                } else {
                    set_statusf("%s timeout 0/%d", label, n);
                    return last_raws[index];
                }
            }
            delay(1);
        }

        total += scales[index].read();
        valid++;

        if (valid % 10 == 0 || valid == n) {
            set_statusf("%s %d/%d", label, valid, n);
        }

        delay(1);
    }

    return total / valid;
}

float raw_to_weight(int index, long raw) {
    if (index < 0 || index >= NB_SCALES) return 0.0f;

    float factor = calibration_factors[index];
    if (factor == 0.0f) return 0.0f;

    float weight = (float)(raw - (long)tare_offsets[index]) / factor;

    if (abs(weight) < ZERO_DEADBAND_G) {
        weight = 0.0f;
    }

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

    if (abs(weight - old) >= DISPLAY_DEADBAND_G) {
        displayed_weights[index] = weight;
    }

    return displayed_weights[index];
}

// ============================================================
// TARE / CALIBRATION
// ============================================================

void tare(int index) {
    if (index < 0 || index >= NB_SCALES) return;

    if (!scale_present[index]) {
        set_statusf("B%d absente", index + 1);
        return;
    }

    set_statusf("Tare B%d: ne touche plus", index + 1);
    delay(1200);

    long raw = read_raw_average_exact(index, TARE_SAMPLES, "Tare");

    last_raws[index] = raw;
    tare_offsets[index] = (float)raw;
    last_weights[index] = 0.0f;
    displayed_weights[index] = 0.0f;
    last_drawn_weight[index] = 999999;
    last_drawn_raw[index] = -999999;

    set_statusf("Tare B%d OK raw=%ld", index + 1, raw);
    update_display_values();
    delay(800);
}

void wait_release_b() {
    while (btn_b.is_pressed()) {
        delay(20);
    }
    delay(DEBOUNCE_MS);
}

void wait_press_b() {
    while (!btn_b.is_pressed()) {
        delay(20);
    }
    delay(DEBOUNCE_MS);
}

void wait_b_click() {
    wait_release_b();
    wait_press_b();
    wait_release_b();
}

void calibrate_0_100g(int index) {
    if (index < 0 || index >= NB_SCALES) return;

    if (!scale_present[index]) {
        set_statusf("B%d absente", index + 1);
        return;
    }

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

    set_statusf("0g OK raw=%ld", raw_0g);
    update_display_values();
    delay(1000);

    set_statusf("Pose %.0fg puis B", CALIBRATION_MASS);
    wait_b_click();

    set_statusf("%.0fg: ne touche plus", CALIBRATION_MASS);
    delay(1500);

    long raw_ref = read_raw_average_exact(index, CALIB_SAMPLES, "Mesure masse");

    last_raws[index] = raw_ref;

    long diff = raw_ref - raw_0g;

    if (abs(diff) < 10) {
        set_statusf("Erreur calib diff=%ld", diff);
        return;
    }

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
// AFFICHAGE ANTI-CLIGNOTEMENT ET ANTI-CRASH
// ============================================================

void reset_draw_cache() {
    for (int i = 0; i < NB_SCALES; i++) {
        last_drawn_weight[i] = 999999;
        last_drawn_raw[i] = -999999;
    }
    last_drawn_active_scale = active_scale;
    last_drawn_mode_focus = mode_focus;
}

void draw_status_bar() {
    tft.fillRect(0, Y_STATUS, 320, 20, TFT_BLACK);
    tft.setTextColor(COLOR_STATUS, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, Y_STATUS);
    tft.print(status_text);
}

void draw_static_screen() {
    // IMPORTANT : cette fonction ne doit JAMAIS appeler update_display_values().
    // Sinon risque de recursion et crash.
    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(COLOR_TITLE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(10, Y_TITLE);

    if (mode_focus) {
        tft.printf("Balance B%d", active_scale + 1);

        tft.setTextColor(COLOR_MODE, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, Y_MODE);
        tft.print("Mode: FOCUS | A tare | B calib | C overview");
    } else {
        tft.print("HX711 x4 SCK D0");

        tft.setTextColor(COLOR_MODE, TFT_BLACK);
        tft.setTextSize(1);
        tft.setCursor(10, Y_MODE);

        if (wifi_connected) {
            tft.print("Mode: OVERVIEW | A tare | B calib | C focus");
        } else {
            tft.print("WiFi ECHEC | Joystick BAS = retry");
        }
    }

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
        // Pas de draw_static_screen() ici : anti-recursion.
        if (!scale_present[active_scale]) {
            tft.setTextColor(COLOR_WHITE, TFT_BLACK);
            tft.setTextSize(4);
            tft.setCursor(10, Y_BIG);
            tft.print("Absente    ");
            return;
        }

        float shown = stable_display_weight(active_scale);
        long raw = last_raws[active_scale];

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
// WIFI
// ============================================================

void setup_wifi() {
    wifi_connected = false;

    Serial.println("Test WiFi Wio Terminal");
    Serial.print("Connexion a : ");
    Serial.println(WIFI_SSID);

    snprintf(status_text, sizeof(status_text), "Connexion WiFi...");
    draw_status_bar();

    WiFi.disconnect();
    delay(200);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int essais = 0;

    while (WiFi.status() != WL_CONNECTED && essais < 30) {
        delay(500);
        Serial.print(".");
        essais++;

        snprintf(status_text, sizeof(status_text), "WiFi... %d/30", essais);
        draw_status_bar();
    }

    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        server.begin();

        Serial.println("WiFi connecte !");
        Serial.print("Adresse IP : ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal RSSI : ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");

        char ip_str[20];
        WiFi.localIP().toString().toCharArray(ip_str, sizeof(ip_str));
        snprintf(status_text, sizeof(status_text), "WiFi OK IP:%s", ip_str);
        draw_status_bar();
    } else {
        wifi_connected = false;

        Serial.println("Echec connexion WiFi");
        Serial.println("Verifie SSID, mot de passe, partage iPhone.");

        snprintf(status_text, sizeof(status_text), "WiFi ECHEC - BAS retry");
        draw_status_bar();
    }
}

void retry_wifi() {
    if (wifi_connected) {
        set_statusf("WiFi deja connecte");
        return;
    }

    set_statusf("Retry WiFi...");
    setup_wifi();
    force_full_redraw();
}

String build_json() {
    String json = "{\"timestamp\":";
    json += String(millis() / 1000);
    json += ",\"balances\":[";

    const char* noms[NB_SCALES] = { "Flacon A", "Flacon B", "Flacon C", "Flacon D" };

    for (int i = 0; i < NB_SCALES; i++) {
        float w = scale_present[i] ? stable_display_weight(i) : 0.0f;

        json += "{\"id\":";
        json += String(i + 1);
        json += ",\"name\":\"";
        json += noms[i];
        json += "\",\"weight\":";
        json += String(w, 4);
        json += "}";

        if (i < NB_SCALES - 1) json += ",";
    }

    json += "]}";
    return json;
}

void handle_http_client() {
    if (!wifi_connected) return;

    WiFiClient client = server.available();
    if (!client) return;

    unsigned long start = millis();
    String request = "";
    while (client.connected() && millis() - start < 200) {
        if (client.available()) {
            char c = client.read();
            request += c;
            if (request.endsWith("\r\n\r\n")) break;
        }
    }

    if (request.startsWith("GET /data")) {
        String json = build_json();

        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: application/json");
        client.println("Access-Control-Allow-Origin: *");
        client.println("Connection: close");
        client.println();
        client.println(json);
    } else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-Type: text/html");
        client.println("Connection: close");
        client.println();
        client.println("<h2>Balances Wio Terminal</h2>");
        client.println("<a href='/data'>GET /data (JSON)</a>");
    }

    delay(1);
    client.stop();
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
        last_weights[i] = 0.0f;
        last_raws[i] = 0;
        displayed_weights[i] = 0.0f;

        if (hx711_detect_dout(DOUT_PINS[i])) {
            scales[i].begin(DOUT_PINS[i], COMMON_SCK_PIN);
            scale_present[i] = true;
            Serial.printf("Balance %d detectee (DOUT=D%d)\n", i + 1, i + 1);
        } else {
            scale_present[i] = false;
            Serial.printf("Balance %d absente\n", i + 1);
        }
    }

    force_full_redraw();
    setup_wifi();
    delay(1000);
    force_full_redraw();
}

// ============================================================
// LOOP
// ============================================================

uint32_t last_display_ms = 0;

void loop() {
    if (btn_a.fell()) {
        tare(active_scale);
    }

    if (btn_b.fell()) {
        calibrate_0_100g(active_scale);
    }

    if (btn_c.fell()) {
        mode_focus = !mode_focus;
        snprintf(status_text, sizeof(status_text),
                 "Mode %s", mode_focus ? "FOCUS" : "OVERVIEW");
        force_full_redraw();
    }

    // Nouvelle option : si WiFi KO, joystick BAS retente la connexion.
    if (btn_down.fell()) {
        retry_wifi();
    }

    if (btn_right.fell()) {
        change_active_scale(1);
        if (mode_focus) {
            force_full_redraw();
        } else {
            update_display_values();
            draw_status_bar();
        }
    }

    if (btn_left.fell()) {
        change_active_scale(-1);
        if (mode_focus) {
            force_full_redraw();
        } else {
            update_display_values();
            draw_status_bar();
        }
    }

    if (mode_focus) {
        read_weight(active_scale);
    } else {
        read_weight(overview_read_index);
        overview_read_index = (overview_read_index + 1) % NB_SCALES;
    }

    if (millis() - last_display_ms >= DISPLAY_PERIOD_MS) {
        update_display_values();
        last_display_ms = millis();
    }

    handle_http_client();

    delay(10);
}
