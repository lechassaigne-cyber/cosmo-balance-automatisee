import time
import board
import digitalio
import displayio
import terminalio
from adafruit_display_text import label
from adafruit_hx711.hx711 import HX711
from adafruit_hx711.analog_in import AnalogIn

# ============================================================
# Wio Terminal + 4 HX711
# Version stabilisee pour test longue duree / derive
#
# A = tare balance active
# B = calibration 0g / 100g balance active
# C court = changer balance active
# C long = changer mode OVERVIEW / FOCUS
#
# Ajouts stabilite :
# - moyenne de lecture plus forte
# - zone morte d'affichage pour eviter les tremblements
# - affichage de l'ecart depuis le debut du test
# - pas de correction automatique de derive
# ============================================================

PIN_CONFIG = [
    (board.D0, board.D1),  # Balance 1 : DT, SCK
    (board.D2, board.D3),  # Balance 2 : DT, SCK
    (board.D4, board.D5),  # Balance 3 : DT, SCK
    (board.D6, board.D7),  # Balance 4 : DT, SCK
]

NB_SCALES = len(PIN_CONFIG)
CALIBRATION_MASS_G = 100.0

# Plus la valeur est haute, plus la mesure est stable mais lente.
# Pour test longue duree, 20 est un bon compromis.
NB_SAMPLES = 20

# Si la variation est plus petite que cette valeur, l'affichage ne bouge pas.
# Cela ne change pas la vraie mesure, seulement l'affichage.
DISPLAY_DEADBAND_G = 0.05

# Delai entre deux rafraichissements.
REFRESH_DELAY_S = 0.25

# Appui long sur C pour changer OVERVIEW / FOCUS.
LONG_PRESS_S = 1.0

# Anti-rebond boutons.
DEBOUNCE_S = 0.20

# ------------------------------------------------------------
# Variables globales
# ------------------------------------------------------------

tare_offsets = [0.0] * NB_SCALES
calibration_factors = [1.0] * NB_SCALES

# Masse mesuree au debut du test, apres tare/calibration.
start_weights = [None] * NB_SCALES

# Derniere valeur affichee, avec zone morte.
displayed_weights = [None] * NB_SCALES

active_scale = 0
mode = "OVERVIEW"
last_button_time = 0.0

# ------------------------------------------------------------
# Boutons
# ------------------------------------------------------------

def make_button(pin):
    btn = digitalio.DigitalInOut(pin)
    btn.direction = digitalio.Direction.INPUT
    btn.pull = digitalio.Pull.UP
    return btn

BTN_A = make_button(board.BUTTON_1)
BTN_B = make_button(board.BUTTON_2)
BTN_C = make_button(board.BUTTON_3)


def pressed(btn):
    return not btn.value


# ------------------------------------------------------------
# Initialisation HX711
# ------------------------------------------------------------

hx_list = []
chan_list = []
data_pins = []
clock_pins = []

for dt_pin, sck_pin in PIN_CONFIG:
    # La librairie Adafruit HX711 attend des objets DigitalInOut,
    # pas directement board.D0, board.D1, etc.
    data = digitalio.DigitalInOut(dt_pin)
    data.direction = digitalio.Direction.INPUT

    clock = digitalio.DigitalInOut(sck_pin)
    clock.direction = digitalio.Direction.OUTPUT

    hx = HX711(data, clock)
    chan = AnalogIn(hx, HX711.CHAN_A_GAIN_128)

    data_pins.append(data)
    clock_pins.append(clock)
    hx_list.append(hx)
    chan_list.append(chan)


# ------------------------------------------------------------
# Lecture stable
# ------------------------------------------------------------

def read_raw_average(index, samples=NB_SAMPLES):
    """Lit plusieurs valeurs brutes HX711 et retourne la moyenne."""
    total = 0
    valid = 0

    for _ in range(samples):
        try:
            total += chan_list[index].value
            valid += 1
        except Exception:
            pass
        time.sleep(0.01)

    if valid == 0:
        return None

    return total / valid


def read_weight(index):
    """Retourne la masse en grammes pour une balance."""
    raw = read_raw_average(index)

    if raw is None:
        return None

    weight = (raw - tare_offsets[index]) / calibration_factors[index]
    return weight


def stable_display_value(index, new_weight):
    """
    Stabilise uniquement l'affichage.
    La vraie mesure reste new_weight.
    """
    if new_weight is None:
        return None

    old = displayed_weights[index]

    if old is None:
        displayed_weights[index] = new_weight
        return new_weight

    if abs(new_weight - old) >= DISPLAY_DEADBAND_G:
        displayed_weights[index] = new_weight

    return displayed_weights[index]


# ------------------------------------------------------------
# Tare et calibration
# ------------------------------------------------------------

def tare_scale(index):
    """Tare la balance active."""
    raw = read_raw_average(index, samples=40)

    if raw is not None:
        tare_offsets[index] = raw
        displayed_weights[index] = 0.0
        start_weights[index] = 0.0


def calibrate_scale_100g(index):
    """
    Calibration simple en 2 etapes :
    1) retirer la masse puis appuyer sur B -> reference 0g
    2) poser 100g puis appuyer encore sur B -> reference 100g
    """
    global calibration_step

    if calibration_step[index] == 0:
        raw_zero = read_raw_average(index, samples=60)
        if raw_zero is not None:
            calibration_zero[index] = raw_zero
            tare_offsets[index] = raw_zero
            calibration_step[index] = 1
            displayed_weights[index] = 0.0
            start_weights[index] = 0.0

    else:
        raw_mass = read_raw_average(index, samples=60)
        raw_zero = calibration_zero[index]

        if raw_mass is not None and raw_zero is not None:
            diff = raw_mass - raw_zero

            if abs(diff) > 1:
                calibration_factors[index] = diff / CALIBRATION_MASS_G
                tare_offsets[index] = raw_zero
                displayed_weights[index] = CALIBRATION_MASS_G
                start_weights[index] = CALIBRATION_MASS_G

        calibration_step[index] = 0


calibration_step = [0] * NB_SCALES
calibration_zero = [None] * NB_SCALES


# ------------------------------------------------------------
# Ecran
# ------------------------------------------------------------

display = board.DISPLAY
screen = displayio.Group()
display.root_group = screen

TITLE_Y = 8
LINE_Y = [35, 58, 81, 104]
INFO_Y = 130

lbl_title = label.Label(terminalio.FONT, text="", x=5, y=TITLE_Y)
screen.append(lbl_title)

lbl_lines = []
for y in LINE_Y:
    l = label.Label(terminalio.FONT, text="", x=5, y=y)
    lbl_lines.append(l)
    screen.append(l)

lbl_info = label.Label(terminalio.FONT, text="", x=5, y=INFO_Y)
screen.append(lbl_info)


def format_weight(value):
    if value is None:
        return "----.--"
    return "{:7.2f}".format(value)


def format_drift(index, value):
    if value is None or start_weights[index] is None:
        return "d: --.--"

    drift = value - start_weights[index]
    return "d:{:+6.2f}".format(drift)


def update_display(weights):
    lbl_title.text = "4 HX711 | mode: {} | B{}".format(mode, active_scale + 1)

    if mode == "OVERVIEW":
        for i in range(NB_SCALES):
            shown = stable_display_value(i, weights[i])
            marker = ">" if i == active_scale else " "
            lbl_lines[i].text = "{}B{} {} g  {}".format(
                marker,
                i + 1,
                format_weight(shown),
                format_drift(i, shown),
            )

        lbl_info.text = "A tare | B calib | C balance | C long mode"

    else:
        shown = stable_display_value(active_scale, weights[active_scale])
        drift_text = format_drift(active_scale, shown)

        lbl_lines[0].text = "Balance active : B{}".format(active_scale + 1)
        lbl_lines[1].text = "Poids : {} g".format(format_weight(shown))
        lbl_lines[2].text = "Ecart debut : {} g".format(
            "--.--" if shown is None or start_weights[active_scale] is None else "{:+.2f}".format(shown - start_weights[active_scale])
        )
        lbl_lines[3].text = "Calib step : {}".format(calibration_step[active_scale])

        lbl_info.text = "A tare | B calib 0/100g | C court/long"


# ------------------------------------------------------------
# Gestion boutons
# ------------------------------------------------------------

def handle_buttons():
    global active_scale, mode, last_button_time

    now = time.monotonic()

    if now - last_button_time < DEBOUNCE_S:
        return

    if pressed(BTN_A):
        tare_scale(active_scale)
        last_button_time = now
        return

    if pressed(BTN_B):
        calibrate_scale_100g(active_scale)
        last_button_time = now
        return

    if pressed(BTN_C):
        press_start = time.monotonic()

        while pressed(BTN_C):
            time.sleep(0.02)

        press_duration = time.monotonic() - press_start

        if press_duration >= LONG_PRESS_S:
            if mode == "OVERVIEW":
                mode = "FOCUS"
            else:
                mode = "OVERVIEW"
        else:
            active_scale = (active_scale + 1) % NB_SCALES

        last_button_time = time.monotonic()
        return


# ------------------------------------------------------------
# Initialisation depart test
# ------------------------------------------------------------

def init_start_weights():
    """
    Memorise les masses de depart pour calculer l'ecart.
    A faire une fois au lancement.
    """
    for i in range(NB_SCALES):
        w = read_weight(i)
        start_weights[i] = w
        displayed_weights[i] = w


# Petite attente pour laisser les HX711 se stabiliser au demarrage.
time.sleep(1.0)
init_start_weights()


# ------------------------------------------------------------
# Boucle principale
# ------------------------------------------------------------

while True:
    handle_buttons()

    weights = []
    for i in range(NB_SCALES):
        weights.append(read_weight(i))

    update_display(weights)
    time.sleep(REFRESH_DELAY_S)
