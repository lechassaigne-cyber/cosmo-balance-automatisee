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
# Version reactive + stable
# Version SCK commun
#
# Branchement actuel :
# - Tous les fils jaunes PD_SCK / SCK -> D0
# - Balance 1 DOUT blanc -> D1
# - Balance 2 DOUT blanc -> D2
# - Balance 3 DOUT blanc -> D3
# - Balance 4 DOUT blanc -> D4
# - Tous les rouges -> 3.3 V
# - Tous les noirs -> GND
#
# A = tare balance active
# B = calibration 0g / 100g balance active
# C = changer mode OVERVIEW / FOCUS
# Joystick gauche/droite = changer balance active
# ============================================================

COMMON_SCK_PIN = board.D0

DOUT_PINS = [
    board.D1,  # Balance 1 : DOUT / DT
    board.D2,  # Balance 2 : DOUT / DT
    board.D3,  # Balance 3 : DOUT / DT
    board.D4,  # Balance 4 : DOUT / DT
]

NB_SCALES = len(DOUT_PINS)
CALIBRATION_MASS_G = 100.0

# Lecture normale plus rapide.
# Si tu veux plus stable mais moins reactif : monte a 8 ou 10.
NB_SAMPLES = 5

# Lecture lente seulement pour tare/calibration.
TARE_SAMPLES = 40
CALIB_SAMPLES = 80

# Frequence affichage.
DISPLAY_PERIOD_S = 0.25

# Anti-rebond boutons.
DEBOUNCE_S = 0.15

# Zone morte autour de zero.
ZERO_DEADBAND_G = 0.05

# Zone morte affichage pour eviter que l'ecran tremble.
DISPLAY_DEADBAND_G = 0.03

# ------------------------------------------------------------
# Variables globales
# ------------------------------------------------------------

tare_offsets = [0.0] * NB_SCALES
calibration_factors = [1.0] * NB_SCALES

active_scale = 0
mode = "OVERVIEW"

last_weights = [None] * NB_SCALES
last_raws = [None] * NB_SCALES
displayed_weights = [None] * NB_SCALES

# En overview, on lit une balance par cycle pour garder les boutons reactifs.
overview_read_index = 0

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

JOY_LEFT = make_button(board.SWITCH_LEFT)
JOY_RIGHT = make_button(board.SWITCH_RIGHT)


def pressed(btn):
    return not btn.value


class ButtonEdge:
    def __init__(self, btn):
        self.btn = btn
        self.last_state = pressed(btn)
        self.last_event_time = 0.0

    def fell(self):
        """Retourne True une seule fois quand le bouton vient d'etre appuye."""
        now = time.monotonic()
        current = pressed(self.btn)

        event = False
        if current and not self.last_state:
            if now - self.last_event_time >= DEBOUNCE_S:
                event = True
                self.last_event_time = now

        self.last_state = current
        return event


btn_a = ButtonEdge(BTN_A)
btn_b = ButtonEdge(BTN_B)
btn_c = ButtonEdge(BTN_C)
btn_left = ButtonEdge(JOY_LEFT)
btn_right = ButtonEdge(JOY_RIGHT)

# ------------------------------------------------------------
# HX711 avec SCK commun
# ------------------------------------------------------------

# IMPORTANT : on cree le SCK une seule fois.
# Il ne faut pas refaire digitalio.DigitalInOut(COMMON_SCK_PIN) pour chaque balance.
common_clock = digitalio.DigitalInOut(COMMON_SCK_PIN)
common_clock.direction = digitalio.Direction.OUTPUT
common_clock.value = False

channels = []
hx711_objects = []
data_objects = []


def hx711_detect_dout(dt_pin, timeout=1.0):
    """Detection simple : DOUT passe a 0 quand le HX711 est pret."""
    data = digitalio.DigitalInOut(dt_pin)
    data.direction = digitalio.Direction.INPUT

    common_clock.value = False
    start = time.monotonic()

    while time.monotonic() - start < timeout:
        if data.value == False:
            data.deinit()
            return True
        time.sleep(0.01)

    data.deinit()
    return False


for dt_pin in DOUT_PINS:
    if hx711_detect_dout(dt_pin):
        data = digitalio.DigitalInOut(dt_pin)
        data.direction = digitalio.Direction.INPUT

        hx711 = HX711(data, common_clock)
        channel = AnalogIn(hx711, HX711.CHAN_A_GAIN_128)

        data_objects.append(data)
        hx711_objects.append(hx711)
        channels.append(channel)
    else:
        data_objects.append(None)
        hx711_objects.append(None)
        channels.append(None)

# ------------------------------------------------------------
# Display
# ------------------------------------------------------------

display = board.DISPLAY
group = displayio.Group()


def mk_label(text, x, y, scale=1, color=0xFFFFFF):
    return label.Label(
        terminalio.FONT,
        text=text,
        x=x,
        y=y,
        scale=scale,
        color=color,
    )


title_lbl = mk_label("HX711 x4 SCK D0", 10, 15, scale=2, color=0x80B0FF)
mode_lbl = mk_label("Mode: OVERVIEW", 10, 45, scale=1, color=0xFFFF80)

line_labels = []
y_positions = [75, 105, 135, 165]

for i in range(NB_SCALES):
    lbl = mk_label(
        "B{}: ----.-- g".format(i + 1),
        10,
        y_positions[i],
        scale=2,
        color=0xFFFFFF,
    )
    line_labels.append(lbl)

big_weight_lbl = mk_label("", 10, 95, scale=4, color=0xFFFFFF)
raw_lbl = mk_label("", 10, 165, scale=1, color=0xA0A0A0)
calib_lbl = mk_label("", 10, 185, scale=1, color=0xFFFF80)
status_lbl = mk_label("A:tare B:calib C:mode Joy:balance", 10, 220, scale=1, color=0x80FF80)

group.append(title_lbl)
group.append(mode_lbl)

for lbl in line_labels:
    group.append(lbl)

group.append(big_weight_lbl)
group.append(raw_lbl)
group.append(calib_lbl)
group.append(status_lbl)

display.root_group = group

# ------------------------------------------------------------
# Mesures
# ------------------------------------------------------------

def read_raw_average(index, n=NB_SAMPLES):
    if channels[index] is None:
        return None

    total = 0
    valid = 0

    for _ in range(n):
        try:
            total += channels[index].value
            valid += 1
        except Exception:
            pass
        time.sleep(0.003)

    if valid == 0:
        return None

    return total / valid


def raw_to_weight(index, raw):
    if raw is None:
        return None

    factor = calibration_factors[index]
    if factor == 0:
        return 0.0

    weight = (raw - tare_offsets[index]) / factor

    if abs(weight) < ZERO_DEADBAND_G:
        weight = 0.0

    return weight


def read_weight_g(index, n=NB_SAMPLES):
    raw = read_raw_average(index, n)
    weight = raw_to_weight(index, raw)
    return weight, raw


def stable_display_weight(index, weight):
    if weight is None:
        return None

    old = displayed_weights[index]
    if old is None:
        displayed_weights[index] = weight
        return weight

    if abs(weight - old) >= DISPLAY_DEADBAND_G:
        displayed_weights[index] = weight

    return displayed_weights[index]

# ------------------------------------------------------------
# Tare / Calibration
# ------------------------------------------------------------

def tare(index):
    if channels[index] is None:
        status_lbl.text = "B{} absente".format(index + 1)
        return

    status_lbl.text = "Tare B{}...".format(index + 1)

    raw = read_raw_average(index, TARE_SAMPLES)
    if raw is not None:
        tare_offsets[index] = raw
        last_weights[index] = 0.0
        displayed_weights[index] = 0.0
        status_lbl.text = "Tare B{} OK".format(index + 1)
    else:
        status_lbl.text = "Erreur tare B{}".format(index + 1)


def wait_release(button):
    while pressed(button):
        time.sleep(0.02)


def wait_press(button):
    while not pressed(button):
        time.sleep(0.02)


def calibrate_0_100g(index):
    if channels[index] is None:
        status_lbl.text = "B{} absente".format(index + 1)
        return

    status_lbl.text = "Calib B{}: retire poids puis B".format(index + 1)
    wait_release(BTN_B)
    wait_press(BTN_B)
    wait_release(BTN_B)

    status_lbl.text = "Mesure 0g..."
    raw_0g = read_raw_average(index, CALIB_SAMPLES)

    if raw_0g is None:
        status_lbl.text = "Erreur 0g B{}".format(index + 1)
        return

    tare_offsets[index] = raw_0g
    displayed_weights[index] = 0.0

    status_lbl.text = "Pose 100g puis B"
    wait_press(BTN_B)
    wait_release(BTN_B)

    status_lbl.text = "Mesure 100g..."
    raw_100g = read_raw_average(index, CALIB_SAMPLES)

    if raw_100g is None:
        status_lbl.text = "Erreur 100g B{}".format(index + 1)
        return

    diff = raw_100g - raw_0g

    if abs(diff) < 1:
        status_lbl.text = "Erreur calib B{}".format(index + 1)
        return

    calibration_factors[index] = diff / CALIBRATION_MASS_G
    last_weights[index] = CALIBRATION_MASS_G
    displayed_weights[index] = CALIBRATION_MASS_G

    status_lbl.text = "Calib B{} OK".format(index + 1)

# ------------------------------------------------------------
# Navigation
# ------------------------------------------------------------

def change_active_scale(direction):
    global active_scale

    active_scale = (active_scale + direction) % NB_SCALES
    status_lbl.text = "Balance active: B{}".format(active_scale + 1)

    if mode == "FOCUS":
        title_lbl.text = "Balance B{}".format(active_scale + 1)

# ------------------------------------------------------------
# Affichage
# ------------------------------------------------------------

def clear_focus_labels():
    big_weight_lbl.text = ""
    raw_lbl.text = ""
    calib_lbl.text = ""


def clear_overview_labels():
    for lbl in line_labels:
        lbl.text = ""


def update_one_scale(index):
    weight, raw = read_weight_g(index)
    last_weights[index] = weight
    last_raws[index] = raw


def update_display_overview():
    title_lbl.text = "HX711 x4 SCK D0"
    mode_lbl.text = "Mode: OVERVIEW | Active B{}".format(active_scale + 1)
    clear_focus_labels()

    for i in range(NB_SCALES):
        if channels[i] is None:
            line_labels[i].text = " B{}: absente".format(i + 1)
            line_labels[i].color = 0x606060
            continue

        shown = stable_display_weight(i, last_weights[i])
        prefix = ">" if i == active_scale else " "

        if shown is None:
            line_labels[i].text = "{}B{}: ----.-- g".format(prefix, i + 1)
        else:
            line_labels[i].text = "{}B{}: {:7.2f} g".format(prefix, i + 1, shown)

        line_labels[i].color = 0x80FF80 if i == active_scale else 0xFFFFFF


def update_display_focus():
    clear_overview_labels()
    title_lbl.text = "Balance B{}".format(active_scale + 1)
    mode_lbl.text = "Mode: FOCUS | Active B{}".format(active_scale + 1)

    if channels[active_scale] is None:
        big_weight_lbl.text = "Absente"
        raw_lbl.text = ""
        calib_lbl.text = ""
        return

    shown = stable_display_weight(active_scale, last_weights[active_scale])
    raw = last_raws[active_scale]

    if shown is None:
        big_weight_lbl.text = "----.-- g"
    else:
        big_weight_lbl.text = "{:.2f} g".format(shown)

    if raw is None:
        raw_lbl.text = "RAW: ----"
    else:
        raw_lbl.text = "RAW: {:.0f}".format(raw)

    calib_lbl.text = "Calib: {:.2f}".format(calibration_factors[active_scale])


def refresh_display():
    if mode == "OVERVIEW":
        update_display_overview()
    else:
        update_display_focus()

# ------------------------------------------------------------
# Demarrage
# ------------------------------------------------------------

time.sleep(1.0)

status_lbl.text = "Demarrage OK - SCK commun sur D0"

last_update = 0.0

# ------------------------------------------------------------
# Boucle principale
# ------------------------------------------------------------

while True:
    now = time.monotonic()

    # 1) Boutons lus en premier, a chaque tour.
    if btn_a.fell():
        tare(active_scale)

    if btn_b.fell():
        calibrate_0_100g(active_scale)

    if btn_c.fell():
        if mode == "OVERVIEW":
            mode = "FOCUS"
            status_lbl.text = "Mode FOCUS"
        else:
            mode = "OVERVIEW"
            status_lbl.text = "Mode OVERVIEW"
        refresh_display()

    if btn_right.fell():
        change_active_scale(1)
        refresh_display()

    if btn_left.fell():
        change_active_scale(-1)
        refresh_display()

    # 2) Lecture des balances.
    # En FOCUS : on lit seulement la balance active, donc c'est rapide.
    # En OVERVIEW : on lit une seule balance par boucle pour ne pas bloquer les boutons.
    # Note : avec SCK commun, lire une balance envoie aussi des impulsions SCK aux autres HX711.
    # Pour 4 balances, ce test permet de valider le cablage.
    # Pour 8 balances, une lecture simultanee personnalisee sera plus propre.
    if mode == "FOCUS":
        update_one_scale(active_scale)
    else:
        update_one_scale(overview_read_index)
        overview_read_index = (overview_read_index + 1) % NB_SCALES

    # 3) Affichage periodique.
    if now - last_update >= DISPLAY_PERIOD_S:
        refresh_display()
        last_update = now

    time.sleep(0.01)
