import time
import board
import digitalio
import displayio
import terminalio
from adafruit_display_text import label
from adafruit_hx711.hx711 import HX711
from adafruit_hx711.analog_in import AnalogIn

# ============================================================
# Wio Terminal + HX711
# A = Tare
# B = Calibration 0g / 100g
# ============================================================

PIN_DT = board.D0
PIN_SCK = board.D1

CALIBRATION_MASS_G = 100.0
CALIBRATION_FACTOR = 1.0
NB_SAMPLES = 15

tare_offset = 0.0

def make_button(pin):
    btn = digitalio.DigitalInOut(pin)
    btn.direction = digitalio.Direction.INPUT
    btn.pull = digitalio.Pull.UP
    return btn

BTN_A = make_button(board.BUTTON_1)
BTN_B = make_button(board.BUTTON_2)

def pressed(btn):
    return not btn.value

data = digitalio.DigitalInOut(PIN_DT)
data.direction = digitalio.Direction.INPUT

clock = digitalio.DigitalInOut(PIN_SCK)
clock.direction = digitalio.Direction.OUTPUT

hx711 = HX711(data, clock)
channel = AnalogIn(hx711, HX711.CHAN_A_GAIN_128)

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

title_lbl = mk_label("Balance HX711", 10, 20, scale=2, color=0x80B0FF)
weight_lbl = mk_label("----.-- g", 10, 80, scale=4, color=0xFFFFFF)
raw_lbl = mk_label("RAW: -----", 10, 135, scale=1, color=0xA0A0A0)
calib_lbl = mk_label("Calib: 1.00", 10, 155, scale=1, color=0xFFFF80)
status_lbl = mk_label("A:tare  B:calib 0/100g", 10, 210, scale=1, color=0x80FF80)

for w in (title_lbl, weight_lbl, raw_lbl, calib_lbl, status_lbl):
    group.append(w)

display.root_group = group

def read_raw_average(n=15):
    total = 0
    for _ in range(n):
        total += channel.value
        time.sleep(0.01)
    return total / n

def tare():
    global tare_offset

    status_lbl.text = "Tare... retire tout poids"
    time.sleep(1.0)

    tare_offset = read_raw_average(40)

    status_lbl.text = "Tare OK"

def read_weight_g():
    raw = read_raw_average(NB_SAMPLES)

    if CALIBRATION_FACTOR == 0:
        return 0.0, raw

    weight = (raw - tare_offset) / CALIBRATION_FACTOR
    return weight, raw

def wait_release(button):
    while pressed(button):
        time.sleep(0.05)

def wait_press(button):
    while not pressed(button):
        time.sleep(0.05)

def calibrate_0_100g():
    global tare_offset, CALIBRATION_FACTOR

    status_lbl.text = "Calib 0g: retire poids"
    weight_lbl.text = "0 g ?"
    time.sleep(2.0)

    raw_0g = read_raw_average(80)
    tare_offset = raw_0g

    status_lbl.text = "Pose 100g puis B"
    weight_lbl.text = "100 g"

    wait_release(BTN_B)
    wait_press(BTN_B)
    wait_release(BTN_B)

    status_lbl.text = "Mesure 100g..."
    time.sleep(1.0)

    raw_100g = read_raw_average(80)
    diff = raw_100g - raw_0g

    if abs(diff) < 1:
        status_lbl.text = "Erreur calibration"
        return

    CALIBRATION_FACTOR = diff / CALIBRATION_MASS_G

    calib_lbl.text = "Calib: {:.2f}".format(CALIBRATION_FACTOR)
    status_lbl.text = "Calibration OK"

time.sleep(1.0)
tare()

last_a = False
last_b = False
last_update = 0.0

while True:
    now = time.monotonic()

    a = pressed(BTN_A)
    b = pressed(BTN_B)

    if a and not last_a:
        tare()

    if b and not last_b:
        calibrate_0_100g()

    last_a = a
    last_b = b

    if now - last_update >= 0.3:
        weight, raw = read_weight_g()

        if abs(weight) < 0.05:
            weight = 0.0

        weight_lbl.text = "{:.2f} g".format(weight)
        raw_lbl.text = "RAW: {:.0f}".format(raw)
        calib_lbl.text = "Calib: {:.2f}".format(CALIBRATION_FACTOR)

        last_update = now

    time.sleep(0.02)