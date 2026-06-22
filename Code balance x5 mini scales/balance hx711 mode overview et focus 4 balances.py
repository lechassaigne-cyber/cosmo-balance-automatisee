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
# A = tare balance active
# B = calibration 0g / 100g balance active
# C = changer mode OVERVIEW / FOCUS
# Joystick gauche/droite = changer balance active
# ============================================================

PIN_CONFIG = [
    (board.D0, board.D1),  # Balance 1
    (board.D2, board.D3),  # Balance 2
    (board.D4, board.D5),  # Balance 3
    (board.D6, board.D7),  # Balance 4
]

NB_SCALES = len(PIN_CONFIG)
CALIBRATION_MASS_G = 100.0
NB_SAMPLES = 8

tare_offsets = [0.0] * NB_SCALES
calibration_factors = [1.0] * NB_SCALES

active_scale = 0
mode = "OVERVIEW"

# ---------------- Detection HX711 ----------------

def hx711_detect(dt_pin, sck_pin, timeout=1.0):
    data = digitalio.DigitalInOut(dt_pin)
    data.direction = digitalio.Direction.INPUT

    clock = digitalio.DigitalInOut(sck_pin)
    clock.direction = digitalio.Direction.OUTPUT
    clock.value = False

    start = time.monotonic()

    while time.monotonic() - start < timeout:
        if data.value == False:
            data.deinit()
            clock.deinit()
            return True
        time.sleep(0.01)

    data.deinit()
    clock.deinit()
    return False

# ---------------- Boutons ----------------

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

# ---------------- HX711 ----------------

channels = []

for dt_pin, sck_pin in PIN_CONFIG:
    if hx711_detect(dt_pin, sck_pin):
        data = digitalio.DigitalInOut(dt_pin)
        data.direction = digitalio.Direction.INPUT

        clock = digitalio.DigitalInOut(sck_pin)
        clock.direction = digitalio.Direction.OUTPUT

        hx711 = HX711(data, clock)
        channel = AnalogIn(hx711, HX711.CHAN_A_GAIN_128)

        channels.append(channel)
    else:
        channels.append(None)

# ---------------- Display ----------------

display = board.DISPLAY
group = displayio.Group()

def mk_label(text, x, y, scale=1, color=0xFFFFFF):
    return label.Label(
        terminalio.FONT,
        text=text,
        x=x,
        y=y,
        scale=scale,
        color=color
    )

title_lbl = mk_label("HX711 x4", 10, 15, scale=2, color=0x80B0FF)
mode_lbl = mk_label("Mode: OVERVIEW", 10, 45, scale=1, color=0xFFFF80)

line_labels = []
y_positions = [75, 105, 135, 165]

for i in range(NB_SCALES):
    lbl = mk_label(
        "B{}: ----.-- g".format(i + 1),
        10,
        y_positions[i],
        scale=2,
        color=0xFFFFFF
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

# ---------------- Mesures ----------------

def read_raw_average(index, n=NB_SAMPLES):
    if channels[index] is None:
        return None

    total = 0
    for _ in range(n):
        total += channels[index].value
        time.sleep(0.005)

    return total / n

def read_weight_g(index):
    raw = read_raw_average(index)

    if raw is None:
        return None, None

    factor = calibration_factors[index]
    if factor == 0:
        return 0.0, raw

    weight = (raw - tare_offsets[index]) / factor

    if abs(weight) < 0.05:
        weight = 0.0

    return weight, raw

def tare(index):
    if channels[index] is None:
        status_lbl.text = "B{} absente".format(index + 1)
        return

    status_lbl.text = "Tare B{}... retire poids".format(index + 1)
    time.sleep(1.0)

    tare_offsets[index] = read_raw_average(index, 40)

    status_lbl.text = "Tare B{} OK".format(index + 1)

def wait_release(button):
    while pressed(button):
        time.sleep(0.05)

def wait_press(button):
    while not pressed(button):
        time.sleep(0.05)

def calibrate_0_100g(index):
    if channels[index] is None:
        status_lbl.text = "B{} absente".format(index + 1)
        return

    status_lbl.text = "Calib B{}: retire poids puis B".format(index + 1)
    wait_release(BTN_B)
    wait_press(BTN_B)
    wait_release(BTN_B)

    status_lbl.text = "Mesure 0g..."
    time.sleep(0.5)

    raw_0g = read_raw_average(index, 80)
    tare_offsets[index] = raw_0g

    status_lbl.text = "Pose 100g puis B"
    wait_press(BTN_B)
    wait_release(BTN_B)

    status_lbl.text = "Mesure 100g..."
    time.sleep(0.5)

    raw_100g = read_raw_average(index, 80)
    diff = raw_100g - raw_0g

    if abs(diff) < 1:
        status_lbl.text = "Erreur calib B{}".format(index + 1)
        return

    calibration_factors[index] = diff / CALIBRATION_MASS_G

    status_lbl.text = "Calib B{} OK".format(index + 1)

# ---------------- Navigation ----------------

def change_active_scale(direction):
    global active_scale

    active_scale = (active_scale + direction) % NB_SCALES
    status_lbl.text = "Balance active: B{}".format(active_scale + 1)

    if mode == "FOCUS":
        title_lbl.text = "Balance B{}".format(active_scale + 1)

# ---------------- Affichage ----------------

def update_display():
    mode_lbl.text = "Mode: {} | Active B{}".format(mode, active_scale + 1)

    if mode == "OVERVIEW":
        title_lbl.text = "HX711 x4"
        big_weight_lbl.text = ""
        raw_lbl.text = ""
        calib_lbl.text = ""

        for i in range(NB_SCALES):
            if channels[i] is None:
                line_labels[i].text = " B{}: absente".format(i + 1)
                line_labels[i].color = 0x606060
                continue

            weight, raw = read_weight_g(i)

            prefix = ">" if i == active_scale else " "
            line_labels[i].text = "{}B{}: {:7.2f} g".format(prefix, i + 1, weight)
            line_labels[i].color = 0x80FF80 if i == active_scale else 0xFFFFFF

    else:
        for lbl in line_labels:
            lbl.text = ""

        title_lbl.text = "Balance B{}".format(active_scale + 1)

        if channels[active_scale] is None:
            big_weight_lbl.text = "Absente"
            raw_lbl.text = ""
            calib_lbl.text = ""
            return

        weight, raw = read_weight_g(active_scale)

        big_weight_lbl.text = "{:.2f} g".format(weight)
        raw_lbl.text = "RAW: {:.0f}".format(raw)
        calib_lbl.text = "Calib: {:.2f}".format(calibration_factors[active_scale])

# ---------------- Demarrage ----------------

time.sleep(1.0)

for i in range(NB_SCALES):
    if channels[i] is not None:
        tare(i)

last_a = False
last_b = False
last_c = False
last_left = False
last_right = False
last_update = 0.0

while True:
    now = time.monotonic()

    a = pressed(BTN_A)
    b = pressed(BTN_B)
    c = pressed(BTN_C)
    left = pressed(JOY_LEFT)
    right = pressed(JOY_RIGHT)

    if a and not last_a:
        tare(active_scale)

    if b and not last_b:
        calibrate_0_100g(active_scale)

    if c and not last_c:
        if mode == "OVERVIEW":
            mode = "FOCUS"
            title_lbl.text = "Balance B{}".format(active_scale + 1)
        else:
            mode = "OVERVIEW"
            title_lbl.text = "HX711 x4"

    if right and not last_right:
        change_active_scale(1)

    if left and not last_left:
        change_active_scale(-1)

    last_a = a
    last_b = b
    last_c = c
    last_left = left
    last_right = right

    if now - last_update >= 0.5:
        update_display()
        last_update = now

    time.sleep(0.02)
