import time
import struct
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
import displayio
import terminalio
from adafruit_display_text import label
import rtc

# ============================================================
#  STAGE LEO - Wio Terminal + MiniScale Unit HX711 via TCA9548A
#
#  Objectif : tester si les MiniScale sont encore exploitables
#
#  Fonctions gardees :
#  - OVERVIEW / FOCUS
#  - Tare simple
#  - Calibration simple 0g / 100g / 1000g
#  - Test LED
#  - Reglage Date / Heure
#  - Logging SD
#
#  Fonctions enlevees :
#  - Aucun reglage EMA / AVG / LPF
#
#  Commandes :
#  - A court        = tare balance active
#  - B court        = calibration 0g / 100g / 1000g
#  - C court        = bascule OVERVIEW / FOCUS
#  - C long         = menu outils
#  - Joystick L/R   = changer balance active
#  - Joystick press = start/stop logging SD
# ============================================================

# -------------------------
# RTC interne
# -------------------------
def set_internal_rtc(year, month, day, hour, minute, second):
    r = rtc.RTC()
    r.datetime = time.struct_time((
        year, month, day,
        hour, minute, second,
        -1, -1, -1
    ))


def load_rtc_from_sd():
    try:
        with open("/sd/rtc.txt", "r") as f:
            data = f.read().strip().split(",")
            year, month, day, hour, minute, second = map(int, data)
            set_internal_rtc(year, month, day, hour, minute, second)
            print("RTC charge depuis SD")
    except Exception as e:
        print("RTC non charge:", e)


# -------------------------
# SD card support
# -------------------------
try:
    import sdcardio
    import storage
    SD_AVAILABLE = True
except Exception:
    SD_AVAILABLE = False


# -------------------------
# Constantes generales
# -------------------------
TARGET_COUNT = 5
SCALE_ADDR = 0x26

# Calibration 0 / 100 / 1000 g
CALIBRATION_MASS_1_G = 100.0
CALIBRATION_MASS_2_G = 1000.0
CALIBRATION_SAMPLES = 80
READ_SAMPLES = 8
SAMPLE_DELAY = 0.01

# Refresh plus lent
DISPLAY_PERIOD = 0.50
MAIN_LOOP_DELAY = 0.05
LOG_PERIOD = 60.0

ZERO_SNAP_THRESHOLD = 0.05
LONG_PRESS_C_TIME = 1.2

LED_CAL_OK = (0, 60, 0)
LED_CAL_NO = (80, 0, 0)
LED_TEST_SEQUENCE = [
    (80, 0, 0),
    (0, 80, 0),
    (0, 0, 80),
    (80, 40, 0),
    (0, 0, 0),
]


# -------------------------
# Modes interface
# -------------------------
MODE_OVERVIEW = 0
MODE_FOCUS = 1
MODE_MENU = 2

MODE_NAMES = {
    MODE_OVERVIEW: "OVERVIEW",
    MODE_FOCUS: "FOCUS",
    MODE_MENU: "MENU",
}


# -------------------------
# Entrees utilisateur
# -------------------------
def _mk_button(pin):
    btn = DigitalInOut(pin)
    btn.direction = Direction.INPUT
    btn.pull = Pull.UP
    return btn


BTN_A = _mk_button(board.BUTTON_1)
BTN_B = _mk_button(board.BUTTON_2)
BTN_C = _mk_button(board.BUTTON_3)
JOY_UP = _mk_button(board.SWITCH_UP)
JOY_DOWN = _mk_button(board.SWITCH_DOWN)
JOY_LEFT = _mk_button(board.SWITCH_LEFT)
JOY_RIGHT = _mk_button(board.SWITCH_RIGHT)
JOY_PRESS = _mk_button(board.SWITCH_PRESS)


def pressed(btn):
    return not btn.value


class Debouncer:
    def __init__(self, delay=0.15):
        self._state = False
        self._last_change = 0.0
        self._delay = delay

    def update(self, value):
        now = time.monotonic()
        if value != self._state and (now - self._last_change) > self._delay:
            self._state = value
            self._last_change = now
            return value
        return False


DB_A = Debouncer()
DB_B = Debouncer()
DB_C = Debouncer()
DB_U = Debouncer()
DB_D = Debouncer()
DB_L = Debouncer()
DB_R = Debouncer()
DB_P = Debouncer()


# -------------------------
# Wrapper TCA9548A
# -------------------------
class MuxChannel:
    def __init__(self, i2c, tca_addr=0x70, channel=0):
        if channel < 0 or channel > 7:
            raise ValueError("Le canal TCA doit etre entre 0 et 7")
        self._i2c = i2c
        self._tca_addr = tca_addr
        self._channel = channel

    def _select(self):
        locked = self._i2c.try_lock()
        try:
            self._i2c.writeto(self._tca_addr, bytes((1 << self._channel,)))
        finally:
            if locked:
                self._i2c.unlock()

    def try_lock(self):
        ok = self._i2c.try_lock()
        if ok:
            self._i2c.writeto(self._tca_addr, bytes((1 << self._channel,)))
        return ok

    def unlock(self):
        self._i2c.unlock()

    def writeto(self, addr, buf, **kwargs):
        self._select()
        try:
            self._i2c.writeto(addr, buf, **kwargs)
        except TypeError:
            self._i2c.writeto(addr, buf)

    def readfrom_into(self, addr, buf):
        self._select()
        self._i2c.readfrom_into(addr, buf)

    def scan(self):
        self._select()
        while not self._i2c.try_lock():
            pass
        try:
            return self._i2c.scan()
        finally:
            self._i2c.unlock()


# -------------------------
# Driver MiniScale simple
# -------------------------
class MiniScaleSimple:
    DEFAULT_ADDR = 0x26

    REG_ADC = 0x00
    REG_WEIGHT = 0x10
    REG_BUTTON = 0x20
    REG_LED = 0x30
    REG_GAP = 0x40
    REG_RESET = 0x50

    def __init__(self, i2c, address=DEFAULT_ADDR, name="", led_order="RGB"):
        addrs = i2c.scan()
        if address not in addrs:
            raise RuntimeError("MiniScale 0x%02X introuvable" % address)

        self.i2c = i2c
        self.addr = address
        self.name = name or "Scale"
        self.led_order = led_order
        self.tare_offset = 0.0
        self.last_weight = 0.0

    def _writeto_mem(self, reg, data):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(self.addr, bytes((reg,)) + data)
        finally:
            self.i2c.unlock()

    def _readfrom_mem(self, reg, nbytes):
        while not self.i2c.try_lock():
            pass
        try:
            try:
                self.i2c.writeto(self.addr, bytes((reg,)), stop=False)
            except TypeError:
                self.i2c.writeto(self.addr, bytes((reg,)))
            buf = bytearray(nbytes)
            self.i2c.readfrom_into(self.addr, buf)
            return bytes(buf)
        finally:
            self.i2c.unlock()

    def read_adc(self):
        return struct.unpack("<I", self._readfrom_mem(self.REG_ADC, 4))[0]

    def read_device_weight(self):
        return struct.unpack("<f", self._readfrom_mem(self.REG_WEIGHT, 4))[0]

    def read_adc_average(self, n=READ_SAMPLES):
        total = 0.0
        for _ in range(n):
            total += float(self.read_adc())
            time.sleep(SAMPLE_DELAY)
        return total / n

    def read_weight_average(self, n=READ_SAMPLES):
        total = 0.0
        for _ in range(n):
            total += float(self.read_device_weight())
            time.sleep(SAMPLE_DELAY)

        weight = (total / n) - self.tare_offset

        if abs(weight) < ZERO_SNAP_THRESHOLD:
            weight = 0.0

        self.last_weight = weight
        return weight

    def tare(self):
        samples = []
        for _ in range(40):
            samples.append(float(self.read_device_weight()))
            time.sleep(SAMPLE_DELAY)
        samples.sort()
        self.tare_offset = samples[len(samples) // 2]
        self.last_weight = 0.0

    def clear_tare(self):
        self.tare_offset = 0.0
        self.last_weight = 0.0

    def reset_internal_offset(self):
        self._writeto_mem(self.REG_RESET, b"\x01")
        time.sleep(0.1)
        self.tare_offset = 0.0
        self.last_weight = 0.0

    def write_gap(self, gap):
        self._writeto_mem(self.REG_GAP, struct.pack("<f", float(gap)))

    def set_led_order(self, order):
        self.led_order = "GRB" if str(order).upper() == "GRB" else "RGB"

    def set_led(self, r, g, b):
        if self.led_order == "GRB":
            payload = bytes((g & 0xFF, r & 0xFF, b & 0xFF))
        else:
            payload = bytes((r & 0xFF, g & 0xFF, b & 0xFF))
        self._writeto_mem(self.REG_LED, payload)

    def read_snapshot(self):
        weight = self.read_weight_average(READ_SAMPLES)
        adc = self.read_adc_average(READ_SAMPLES)
        return {
            "weight": weight,
            "adc": adc,
        }


# -------------------------
# Affichage
# -------------------------
display = board.DISPLAY
group = displayio.Group()
display.auto_refresh = True


def mk_label(text, x, y, scale=1, color=0xFFFFFF):
    return label.Label(terminalio.FONT, text=text, x=x, y=y, scale=scale, color=color)


title_lbl = mk_label("Stage Leo - MiniScale", 6, 18, scale=2, color=0x80B0FF)
mode_lbl = mk_label("Mode: ----", 6, 38, scale=1, color=0x80B0FF)
date_lbl = mk_label("Date: ----/--/--", 6, 52, scale=1, color=0xFFFF80)
time_lbl = mk_label("Heure: --:--:--", 160, 52, scale=1, color=0xFFFF80)
focus_lbl = mk_label("S-  ----.-- g", 6, 76, scale=3, color=0xFFFFFF)
adc_lbl = mk_label("ADC: -----", 6, 115, scale=1, color=0xA0A0A0)
info_lbl = mk_label("", 6, 140, scale=1, color=0x80FF80)
status_lbl = mk_label("", 6, 165, scale=1, color=0xFFB070)
menu_lbl = mk_label("", 6, 188, scale=1, color=0xFFFFFF)
hint_lbl = mk_label("L/R:sel A:tare B:cal C:mode C long:menu", 6, 224, scale=1, color=0x80B0FF)

for widget in (title_lbl, mode_lbl, date_lbl, time_lbl, focus_lbl, adc_lbl, info_lbl, status_lbl, menu_lbl, hint_lbl):
    group.append(widget)

display.root_group = group


# -------------------------
# I2C / TCA detection
# -------------------------
root_i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)


def detect_tca_addr(i2c):
    while not i2c.try_lock():
        pass
    try:
        addrs = set(i2c.scan())
    finally:
        i2c.unlock()

    for addr in range(0x70, 0x78):
        if addr in addrs:
            return addr
    raise RuntimeError("TCA9548A introuvable")


MUX_ADDR = detect_tca_addr(root_i2c)


def tca_write_mask(mask):
    while not root_i2c.try_lock():
        pass
    try:
        root_i2c.writeto(MUX_ADDR, bytes((mask,)))
    finally:
        root_i2c.unlock()


def tca_select_channel(ch):
    tca_write_mask(1 << ch)
    time.sleep(0.005)


def probe_miniscale_on_current_channel(addr=SCALE_ADDR):
    for _ in range(3):
        try:
            while not root_i2c.try_lock():
                pass
            try:
                try:
                    root_i2c.writeto(addr, bytes((0x10,)), stop=False)
                except TypeError:
                    root_i2c.writeto(addr, bytes((0x10,)))
                buf = bytearray(4)
                root_i2c.readfrom_into(addr, buf)
                return True
            finally:
                root_i2c.unlock()
        except Exception:
            time.sleep(0.04)
    return False


def detect_scale_channels(addr=SCALE_ADDR):
    found = []
    tca_write_mask(0x00)
    time.sleep(0.02)

    for ch in range(8):
        tca_select_channel(ch)
        if probe_miniscale_on_current_channel(addr):
            found.append(ch)

    tca_write_mask(0x00)
    return found


# -------------------------
# Logging CSV
# -------------------------
class CsvLogger:
    def __init__(self):
        self.enabled = False
        self.file = None
        self.last_log = 0.0
        self._mounted = False
        self.current_path = None
        self.pending_lines = 0
        self.last_flush = 0.0
        self.spi = busio.SPI(board.SD_SCK, board.SD_MOSI, board.SD_MISO)
        self.sd = None

    def sd_mount(self):
        if not SD_AVAILABLE:
            raise RuntimeError("sdcardio/storage indisponibles")
        if self._mounted:
            return
        self.sd = sdcardio.SDCard(self.spi, board.SD_CS)
        vfs = storage.VfsFat(self.sd)
        storage.mount(vfs, "/sd")
        self._mounted = True

    def _build_log_filename(self, nb_scales):
        tm = time.localtime()
        date_str = "{:04d}-{:02d}-{:02d}".format(tm.tm_year, tm.tm_mon, tm.tm_mday)
        heure_str = "{:02d}h{:02d}m{:02d}s".format(tm.tm_hour, tm.tm_min, tm.tm_sec)
        return "/sd/cosmo_evap_x{}_{}_{}.csv".format(nb_scales, date_str, heure_str)

    def open(self, nb_scales):
        self.sd_mount()
        self.current_path = self._build_log_filename(nb_scales)
        self.file = open(self.current_path, "w")

        headers = ["Date", "Heure"]
        for ch in range(8):
            headers.append("Ch{}".format(ch))
        self.file.write(";".join(headers) + "\n")

        self.enabled = True
        self.last_log = 0.0
        self.pending_lines = 0
        self.last_flush = time.monotonic()

    def close(self):
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
        self.file = None
        self.current_path = None
        self.enabled = False
        self.pending_lines = 0

    def write_snapshots(self, now_s, snapshots):
        if not self.enabled or not self.file:
            return
        if (now_s - self.last_log) < LOG_PERIOD:
            return

        tm = time.localtime()
        date_str = "{:02d}/{:02d}/{:04d}".format(tm.tm_mday, tm.tm_mon, tm.tm_year)
        heure_str = "{:02d}:{:02d}:{:02d}".format(tm.tm_hour, tm.tm_min, tm.tm_sec)

        row = [date_str, heure_str]
        for ch in range(8):
            snap = snapshots.get(ch)
            if snap is None:
                poids = ""
            elif "error" in snap:
                poids = "Erreur"
            else:
                poids = "{:.2f}".format(snap["weight"]).replace(".", ",")
            row.append(poids)

        try:
            self.file.write(";".join(row) + "\n")
            self.pending_lines += 1
            self.last_log = now_s

            if self.pending_lines >= 5 or (now_s - self.last_flush) >= 5.0:
                self.file.flush()
                self.pending_lines = 0
                self.last_flush = now_s
        except Exception as e:
            print("Erreur ecriture SD:", e)


# -------------------------
# Application principale
# -------------------------
class ScaleApp:
    def __init__(self):
        self.mode = MODE_OVERVIEW
        self.previous_mode = MODE_OVERVIEW
        self.scales = []
        self.active_idx = -1
        self.logger = CsvLogger()
        self.last_display = 0.0
        self.last_snapshots = {}
        self.c_press_start = None
        self.c_long_done = False

        try:
            self.logger.sd_mount()
            load_rtc_from_sd()
        except Exception as e:
            print("SD non montee au demarrage:", e)

        self.init_scales()

    def set_status(self, text):
        status_lbl.text = text

    def init_scales(self):
        all_found = detect_scale_channels()
        channels = all_found[:TARGET_COUNT] if len(all_found) >= TARGET_COUNT else all_found[:]
        self.rebuild_scales(channels)
        self.set_status("Init OK: {} balance(s)".format(len(self.scales)))

    def rebuild_scales(self, channels):
        new_scales = []

        for ch in channels:
            mux_bus = MuxChannel(root_i2c, tca_addr=MUX_ADDR, channel=ch)
            try:
                sc = MiniScaleSimple(mux_bus, name="S{}".format(ch), led_order="RGB")
                sc.set_led(*LED_CAL_NO)
                new_scales.append({
                    "ch": ch,
                    "scale": sc,
                    "led_order": "RGB",
                    "calibrated": False,
                })
            except Exception as e:
                print("Erreur canal {}: {}".format(ch, e))

        self.scales = new_scales
        self.active_idx = 0 if self.scales else -1

    def active_entry(self):
        if self.active_idx < 0 or self.active_idx >= len(self.scales):
            return None
        return self.scales[self.active_idx]

    def active_scale(self):
        entry = self.active_entry()
        return entry["scale"] if entry else None

    def next_active(self, delta):
        if not self.scales:
            return
        self.active_idx = (self.active_idx + delta) % len(self.scales)
        self.mode = MODE_FOCUS

    def toggle_mode(self):
        if self.mode == MODE_OVERVIEW:
            self.mode = MODE_FOCUS
        else:
            self.mode = MODE_OVERVIEW

    def build_snapshots(self):
        snapshots = {}
        for entry in self.scales:
            ch = entry["ch"]
            sc = entry["scale"]
            try:
                snapshots[ch] = sc.read_snapshot()
            except Exception as e:
                snapshots[ch] = {"error": str(e)}
        return snapshots

    def update_display(self, snapshots):
        entry = self.active_entry()
        mode_lbl.text = "Mode: {}".format(MODE_NAMES.get(self.mode, "?"))

        tm = time.localtime()
        date_lbl.text = "{:04d}-{:02d}-{:02d}".format(tm.tm_year, tm.tm_mon, tm.tm_mday)
        time_lbl.text = "{:02d}:{:02d}:{:02d}".format(tm.tm_hour, tm.tm_min, tm.tm_sec)

        title_lbl.text = "MiniScale x{} Log:{} SD:{}".format(
            len(self.scales),
            "ON" if self.logger.enabled else "OFF",
            "ON" if self.logger._mounted else "OFF"
        )

        menu_lbl.text = ""

        if not entry:
            focus_lbl.scale = 2
            focus_lbl.text = "Aucune balance"
            adc_lbl.text = ""
            info_lbl.text = "Verifier TCA / alim / cablage"
            return

        ch = entry["ch"]
        snap = snapshots.get(ch)

        if self.mode == MODE_FOCUS:
            focus_lbl.scale = 3
            focus_lbl.y = 76
            adc_lbl.y = 115

            if not snap or "error" in snap:
                focus_lbl.text = "S{} ERREUR".format(ch)
                adc_lbl.text = "ADC: -----"
                info_lbl.text = "Erreur I2C"
            else:
                focus_lbl.text = "S{} {:7.2f} g".format(ch, snap["weight"])
                adc_lbl.text = "ADC: {:.0f}".format(snap["adc"])
                info_lbl.text = "Cal:{}".format("OK" if entry["calibrated"] else "NO")

        elif self.mode == MODE_OVERVIEW:
            focus_lbl.scale = 1
            focus_lbl.y = 76
            adc_lbl.y = 96

            line1 = []
            line2 = []
            for ch_i in range(8):
                snap_i = snapshots.get(ch_i)
                if snap_i is None:
                    txt = "S{}:---".format(ch_i)
                elif "error" in snap_i:
                    txt = "S{}:Err".format(ch_i)
                else:
                    txt = "S{}:{:.1f}".format(ch_i, snap_i["weight"])

                if ch_i < 4:
                    line1.append(txt)
                else:
                    line2.append(txt)

            focus_lbl.text = " ".join(line1)
            adc_lbl.text = " ".join(line2)
            info_lbl.text = "Vue globale - C:focus"

    def toggle_logging(self):
        if not self.logger.enabled:
            try:
                self.logger.open(len(self.scales))
                self.set_status("Logging ON")
            except Exception as e:
                self.set_status("Log err: {}".format(e))
        else:
            self.logger.close()
            self.set_status("Logging OFF")

    def wait_release(self, button):
        while pressed(button):
            time.sleep(0.05)

    def wait_press(self, button):
        while not pressed(button):
            time.sleep(0.05)

    def read_adc_average_for_calib(self, scale, n=CALIBRATION_SAMPLES):
        total = 0.0
        for _ in range(n):
            total += float(scale.read_adc())
            time.sleep(SAMPLE_DELAY)
        return total / n

    def tare_active(self):
        sc = self.active_scale()
        entry = self.active_entry()
        if not sc or not entry:
            return

        self.set_status("[S{}] Tare... retire poids".format(entry["ch"]))
        time.sleep(1.0)
        sc.tare()
        self.set_status("[S{}] Tare OK".format(entry["ch"]))

    def calibrate_active_0_100_1000(self):
        sc = self.active_scale()
        entry = self.active_entry()
        if not sc or not entry:
            return

        ch = entry["ch"]
        self.mode = MODE_FOCUS

        self.set_status("[S{}] Calib 0g: retire poids".format(ch))
        focus_lbl.text = "0 g ?"
        time.sleep(2.0)

        sc.reset_internal_offset()
        raw_0g = self.read_adc_average_for_calib(sc, CALIBRATION_SAMPLES)
        sc.tare_offset = 0.0

        self.set_status("[S{}] Pose 100g puis B".format(ch))
        focus_lbl.text = "100 g"
        self.wait_release(BTN_B)
        self.wait_press(BTN_B)
        self.wait_release(BTN_B)

        self.set_status("[S{}] Mesure 100g...".format(ch))
        time.sleep(1.0)
        raw_100g = self.read_adc_average_for_calib(sc, CALIBRATION_SAMPLES)

        self.set_status("[S{}] Pose 1000g puis B".format(ch))
        focus_lbl.text = "1000 g"
        self.wait_press(BTN_B)
        self.wait_release(BTN_B)

        self.set_status("[S{}] Mesure 1000g...".format(ch))
        time.sleep(1.0)
        raw_1000g = self.read_adc_average_for_calib(sc, CALIBRATION_SAMPLES)

        diff_100 = raw_100g - raw_0g
        diff_1000 = raw_1000g - raw_0g

        if abs(diff_100) < 1 or abs(diff_1000) < 1:
            self.set_status("[S{}] Erreur calibration".format(ch))
            info_lbl.text = "Diff trop faible"
            sc.set_led(*LED_CAL_NO)
            time.sleep(2.0)
            return

        w1 = CALIBRATION_MASS_1_G
        w2 = CALIBRATION_MASS_2_G
        gap = (w1 * diff_100 + w2 * diff_1000) / (w1 * w1 + w2 * w2)
        sc.write_gap(gap)

        self.set_status("[S{}] Tare finale... retire poids".format(ch))
        focus_lbl.text = "Retire poids"
        time.sleep(2.0)
        sc.tare()

        entry["calibrated"] = True
        sc.set_led(*LED_CAL_OK)

        self.set_status("[S{}] Calibration OK".format(ch))
        info_lbl.text = "GAP:{:.5f}".format(gap)
        time.sleep(2.0)

    def led_test_active(self):
        sc = self.active_scale()
        entry = self.active_entry()
        if not sc or not entry:
            return

        self.set_status("[S{}] Test LED".format(entry["ch"]))
        for color in LED_TEST_SEQUENCE:
            sc.set_led(*color)
            time.sleep(0.25)

        if entry["calibrated"]:
            sc.set_led(*LED_CAL_OK)
        else:
            sc.set_led(*LED_CAL_NO)

        self.set_status("Test LED fini")

    def clear_tare_active(self):
        sc = self.active_scale()
        entry = self.active_entry()
        if sc and entry:
            sc.clear_tare()
            self.set_status("[S{}] Tare effacee".format(entry["ch"]))
            time.sleep(0.6)

    def datetime_menu(self):
        tm = time.localtime()
        year = tm.tm_year
        month = tm.tm_mon
        day = tm.tm_mday
        hour = tm.tm_hour
        minute = tm.tm_min
        second = tm.tm_sec

        items = ["Annee", "Mois", "Jour", "Heure", "Minute", "Seconde", "Valider", "Annuler"]
        sel = 0

        while True:
            mode_lbl.text = "Mode: RTC"
            focus_lbl.scale = 1
            focus_lbl.text = "Reglage Date / Heure"
            adc_lbl.text = "> {}".format(items[sel])
            info_lbl.text = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
                year, month, day, hour, minute, second
            )
            status_lbl.text = "UP/DOWN:ligne L/R:val B:OK C:retour"
            menu_lbl.text = ""

            if DB_U.update(pressed(JOY_UP)):
                sel = (sel - 1) % len(items)
            if DB_D.update(pressed(JOY_DOWN)):
                sel = (sel + 1) % len(items)

            if sel == 0:
                if DB_L.update(pressed(JOY_LEFT)):
                    year = max(2024, year - 1)
                if DB_R.update(pressed(JOY_RIGHT)):
                    year = min(2099, year + 1)
            elif sel == 1:
                if DB_L.update(pressed(JOY_LEFT)):
                    month = 12 if month <= 1 else month - 1
                if DB_R.update(pressed(JOY_RIGHT)):
                    month = 1 if month >= 12 else month + 1
            elif sel == 2:
                if DB_L.update(pressed(JOY_LEFT)):
                    day = 31 if day <= 1 else day - 1
                if DB_R.update(pressed(JOY_RIGHT)):
                    day = 1 if day >= 31 else day + 1
            elif sel == 3:
                if DB_L.update(pressed(JOY_LEFT)):
                    hour = 23 if hour <= 0 else hour - 1
                if DB_R.update(pressed(JOY_RIGHT)):
                    hour = 0 if hour >= 23 else hour + 1
            elif sel == 4:
                if DB_L.update(pressed(JOY_LEFT)):
                    minute = 59 if minute <= 0 else minute - 1
                if DB_R.update(pressed(JOY_RIGHT)):
                    minute = 0 if minute >= 59 else minute + 1
            elif sel == 5:
                if DB_L.update(pressed(JOY_LEFT)):
                    second = 59 if second <= 0 else second - 1
                if DB_R.update(pressed(JOY_RIGHT)):
                    second = 0 if second >= 59 else second + 1
            elif sel == 6:
                if DB_B.update(pressed(BTN_B)) or DB_P.update(pressed(JOY_PRESS)):
                    try:
                        set_internal_rtc(year, month, day, hour, minute, second)
                        try:
                            if self.logger._mounted:
                                with open("/sd/rtc.txt", "w") as f:
                                    f.write("{},{},{},{},{},{}".format(year, month, day, hour, minute, second))
                        except Exception as e:
                            print("Erreur sauvegarde RTC:", e)
                        self.set_status("RTC mise a jour")
                        time.sleep(0.8)
                        return
                    except Exception as e:
                        self.set_status("RTC err: {}".format(e))
                        time.sleep(1.0)
            elif sel == 7:
                if DB_B.update(pressed(BTN_B)):
                    return

            if DB_C.update(pressed(BTN_C)):
                return

            time.sleep(0.05)

    def tools_menu(self):
        old_mode = self.mode
        self.mode = MODE_MENU
        items = ["Test LED", "Date / Heure", "Clear tare", "Rescan balances", "Quitter"]
        sel = 0

        self.wait_release(BTN_C)

        while True:
            mode_lbl.text = "Mode: MENU"
            focus_lbl.scale = 1
            focus_lbl.text = "Menu outils"
            adc_lbl.text = "> {}".format(items[sel])
            info_lbl.text = "UP/DOWN choisir  B valider  C retour"
            menu_lbl.text = ""

            if DB_U.update(pressed(JOY_UP)):
                sel = (sel - 1) % len(items)
            if DB_D.update(pressed(JOY_DOWN)):
                sel = (sel + 1) % len(items)

            if DB_B.update(pressed(BTN_B)) or DB_P.update(pressed(JOY_PRESS)):
                if sel == 0:
                    self.led_test_active()
                elif sel == 1:
                    self.datetime_menu()
                elif sel == 2:
                    self.clear_tare_active()
                elif sel == 3:
                    self.set_status("Rescan TCA...")
                    self.init_scales()
                    time.sleep(0.8)
                elif sel == 4:
                    self.mode = old_mode
                    return

            if DB_C.update(pressed(BTN_C)):
                self.mode = old_mode
                return

            time.sleep(0.05)

    def handle_c_button(self):
        now = time.monotonic()

        if pressed(BTN_C):
            if self.c_press_start is None:
                self.c_press_start = now
                self.c_long_done = False
            elif not self.c_long_done and (now - self.c_press_start) >= LONG_PRESS_C_TIME:
                self.c_long_done = True
                self.tools_menu()
        else:
            if self.c_press_start is not None:
                if not self.c_long_done:
                    self.toggle_mode()
                self.c_press_start = None
                self.c_long_done = False

    def handle_buttons(self):
        if DB_L.update(pressed(JOY_LEFT)):
            self.next_active(-1)

        if DB_R.update(pressed(JOY_RIGHT)):
            self.next_active(+1)

        if DB_P.update(pressed(JOY_PRESS)):
            self.toggle_logging()

        if DB_A.update(pressed(BTN_A)):
            self.tare_active()

        if DB_B.update(pressed(BTN_B)):
            self.calibrate_active_0_100_1000()

        self.handle_c_button()

    def loop(self):
        while True:
            now = time.monotonic()
            self.handle_buttons()

            if now - self.last_display >= DISPLAY_PERIOD:
                self.last_snapshots = self.build_snapshots()
                self.update_display(self.last_snapshots)
                self.logger.write_snapshots(now, self.last_snapshots)
                self.last_display = now

            time.sleep(MAIN_LOOP_DELAY)


# -------------------------
# Lancement
# -------------------------
set_internal_rtc(2026, 4, 14, 12, 0, 0)

app = ScaleApp()
time.sleep(1.0)
app.loop()
