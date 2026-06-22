# Migration Python → C++ : tableau de correspondance

Ce guide montre ligne par ligne comment ton code Python a été traduit en C++.

---

## Correspondances directes Python → C++

| Python                              | C++                                              |
|-------------------------------------|--------------------------------------------------|
| `time.monotonic()`                  | `millis()` (en millisecondes)                    |
| `time.sleep(0.01)`                  | `delay(10)`                                      |
| `DISPLAY_PERIOD_S = 0.25`           | `#define DISPLAY_PERIOD_MS 250`                  |
| `DEBOUNCE_S = 0.15`                 | `#define DEBOUNCE_MS 150`                        |
| `[0.0] * NB_SCALES`                 | `float arr[NB_SCALES] = {0.0f, ...}`             |
| `btn.pull = Pull.UP`                | `pinMode(pin, INPUT_PULLUP)`                     |
| `not btn.value`                     | `digitalRead(pin) == LOW`                        |
| `class ButtonEdge:`                 | `struct ButtonEdge { ... }`                      |
| `def fell(self):`                   | `bool fell() { ... }`                            |
| `digitalio.DigitalInOut(pin)`       | `pinMode(pin, OUTPUT)`                           |
| `common_clock.value = False`        | `digitalWrite(COMMON_SCK_PIN, LOW)`              |
| `hx711 = HX711(data, clock)`        | `scales[i].begin(DOUT_PINS[i], COMMON_SCK_PIN)` |
| `channel.value`                     | `scales[i].read()`                               |
| `Serial.println(...)`               | `Serial.println(...)` (identique!)               |
| `"B{}: {:7.2f} g".format(i, v)`     | `printf("B%d: %7.2f g", i, v)`                  |
| `abs(weight) < ZERO_DEADBAND_G`     | `abs(weight) < ZERO_DEADBAND_G`  (identique)     |
| `mode = "OVERVIEW"` / `"FOCUS"`     | `bool mode_focus = false` / `true`               |
| `active_scale = (active_scale + 1) % NB_SCALES` | identique en C++              |

---

## Différences importantes à connaître

### 1. Les types de données sont explicites en C++

```python
# Python : type automatique
poids = 48.32
nb = 5
```

```cpp
// C++ : tu déclares le type
float poids = 48.32f;   // le 'f' dit que c'est un float (pas double)
int   nb    = 5;
```

### 2. Les tableaux ont une taille fixe

```python
# Python : liste dynamique, peut grandir
tare_offsets = [0.0] * 4
tare_offsets.append(0.0)  # possible
```

```cpp
// C++ : taille fixe déclarée à la création
float tare_offsets[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// impossible d'ajouter des éléments après !
```

### 3. Les chaînes de caractères

```python
# Python : format() ou f-strings
msg = "B{}: {:7.2f} g".format(i + 1, poids)
msg = f"B{i+1}: {poids:.2f} g"
```

```cpp
// C++ : snprintf (pour char[]) ou printf/tft.printf
char msg[32];
snprintf(msg, sizeof(msg), "B%d: %7.2f g", i + 1, poids);

// ou directement sur l'écran :
tft.printf("B%d: %7.2f g", i + 1, poids);
```

### 4. Les classes → structures (struct)

```python
# Python
class ButtonEdge:
    def __init__(self, btn):
        self.btn = btn
        self.last_state = pressed(btn)

    def fell(self):
        ...

btn_a = ButtonEdge(BTN_A)
```

```cpp
// C++
struct ButtonEdge {
    int  pin;
    bool last_state;

    void begin(int p) { ... }  // équivalent de __init__
    bool fell() { ... }
};

ButtonEdge btn_a;
btn_a.begin(WIO_KEY_A);
```

### 5. millis() vs time.monotonic()

```python
# Python : secondes flottantes
now = time.monotonic()            # ex: 123.456
if now - last_update >= 0.25:    # 0.25 secondes
    last_update = now
```

```cpp
// C++ : millisecondes entières
uint32_t now = millis();               // ex: 123456
if (now - last_display_ms >= 250) {   // 250 ms
    last_display_ms = now;
}
```

### 6. #define vs les constantes Python

```python
# Python
NB_SAMPLES = 5
ZERO_DEADBAND_G = 0.05
```

```cpp
// C++ : deux façons
#define NB_SAMPLES 5              // remplacement textuel avant compilation
const float ZERO_DEADBAND_G = 0.05f;  // vraie constante typée (préférable)
```

---

## Structure des bibliothèques à installer

### Dans Arduino IDE :
1. Ouvre Outils → Gérer les bibliothèques
2. Installe :
   - `HX711 by bogde`
   - `Seeed Arduino TFT_eSPI` (déjà incluse si tu as configuré le Wio Terminal)
   - `Seeed Arduino rpcWiFi` (idem)

### Dans PlatformIO (VS Code) :
Tout est dans platformio.ini, c'est automatique.

---

## Pour tester étape par étape

1. **D'abord sans WiFi** : commente `setup_wifi()` dans `setup()` et teste que les balances fonctionnent exactement comme en Python.

2. **Ensuite ajoute le WiFi** : décommente `setup_wifi()`, mets tes identifiants, et vérifie l'IP dans le Serial Monitor.

3. **Test depuis navigateur** : ouvre `http://<ip>/data` dans Safari sur iPhone → tu dois voir le JSON.

4. **Enfin l'app iOS** : entre l'IP dans les réglages de l'app.
