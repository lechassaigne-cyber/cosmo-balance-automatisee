import time
import math
import struct
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
import displayio
import terminalio
from adafruit_display_text import label

# ============================================================
#  STAGE LEO - Wio Terminal + 5 balances MiniScale via TCA9548A
#  Version UI améliorée
#  Même logique / mêmes fonctionnalités
#  Interface seulement retravaillée
# ============================================================

try:
    import sdcardio
    import storage
    SD_AVAILABLE = True
except Exception:
    SD_AVAILABLE = False

TARGET_COUNT = 5
SCALE_ADDR = 0x26
LOG_PATH = "/sd/evaporation_log.csv"

DEFAULT_ALPHA = 8
DEFAULT_AVG = 50
DEFAULT_LPF = True

MEDIAN_FILTER_SIZE = 5
STABILITY_WINDOW = 20
STABILITY_THRESHOLD = 0.2
BASELINE_DRIFT_WINDOW = 100
OUTLIER_REJECT_SIGMA = 4.0

SAMPLES_PER_POINT = 500
SPS_HINT = 10
SAMPLE_DELAY = 1.0 / SPS_HINT
LOG_PERIOD = 1.0

LED_CAL_OK = (0, 60, 0)
LED_CAL_NO = (80, 0, 0)
LED_TEST_SEQUENCE = [
    (0, 30, 10),
    (20, 0, 80),
    (80, 20, 0),
    (0, 0, 0),
]

COLOR_BG = 0x0B0F14
COLOR_ACCENT = 0x33D1FF
COLOR_TITLE = 0xB8F3FF
COLOR_TEXT = 0xF2F7FF
COLOR_SUB = 0xA9B8C8
COLOR_OK = 0x73FFA3
COLOR_WARN = 0xFFC46B
COLOR_ERR = 0xFF7A7A
COLOR_DIM = 0x6C7A89

MODE_OVERVIEW = 0
MODE_FOCUS = 1
MODE_SETTINGS_SCALE = 2
MODE_SETTINGS_ALL = 3
MODE_CALIBRATION = 4

MODE_NAMES = {
    MODE_OVERVIEW: "OVERVIEW",
    MODE_FOCUS: "FOCUS",
    MODE_SETTINGS_SCALE: "SET SCALE",
    MODE_SETTINGS_ALL: "SET ALL",
    MODE_CALIBRATION: "CALIB",
}

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

class MedianFilter:
    def __init__(self, size=5):
        self.size = size
        self.buffer = []

    def clear(self):
        self.buffer = []

    def add(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.size:
            self.buffer.pop(0)

    def get(self):
        if not self.buffer:
            return 0.0
        arr = sorted(self.buffer)
        mid = len(arr) // 2
        if len(arr) % 2 == 0:
            return (arr[mid - 1] + arr[mid]) / 2.0
        return arr[mid]

class StabilityDetector:
    def __init__(self, threshold=0.2, required_samples=20):
        self.threshold = threshold
        self.required_samples = required_samples
        self.buffer = []

    def clear(self):
        self.buffer = []

    def add(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.required_samples:
            self.buffer.pop(0)

    def is_stable(self):
        if len(self.buffer) < self.required_samples:
            return False
        mean = sum(self.buffer) / len(self.buffer)
        max_dev = max(abs(v - mean) for v in self.buffer)
        return max_dev < self.threshold

    def get_stable_value(self):
        if not self.is_stable():
            return None
        return sum(self.buffer) / len(self.buffer)

class BaselineDriftMonitor:
    def __init__(self, window=100):
        self.window = window
        self.history = []
        self.estimated_drift = 0.0

    def clear(self):
        self.history = []
        self.estimated_drift = 0.0

    def add_reading(self, value):
        self.history.append(value)
        if len(self.history) > self.window:
            self.history.pop(0)

        if len(self.history) >= 20:
            recent = self.history[-20:]
            old = self.history[:20] if len(self.history) >= 40 else self.history[:10]
            avg_recent = sum(recent) / len(recent)
            avg_old = sum(old) / len(old)
            self.estimated_drift = avg_recent - avg_old

    def get_drift_estimate(self):
        return self.estimated_drift

class MuxChannel:
    def __init__(self, i2c, tca_addr=0x70, channel=0):
        if channel < 0 or channel > 7:
            raise ValueError("Le canal TCA doit être entre 0 et 7")
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

class MiniScaleEnhanced:
    DEFAULT_ADDR = 0x26
    REG_ADC = 0x00
    REG_WEIGHT = 0x10
    REG_BUTTON = 0x20
    REG_LED = 0x30
    REG_GAP = 0x40
    REG_RESET = 0x50
    REG_LPF = 0x80
    REG_AVG = 0x81
    REG_EMA = 0x82

    def __init__(self, i2c, address=DEFAULT_ADDR, name="", led_order="RGB"):
        addrs = i2c.scan()
        if address not in addrs:
            raise RuntimeError("MiniScale 0x%02X introuvable" % address)

        self.i2c = i2c
        self.addr = address
        self.name = name or "Scale"
        self.led_order = led_order
        self._tare_offset = 0.0

        self.median_filter = MedianFilter(MEDIAN_FILTER_SIZE)
        self.stability = StabilityDetector(STABILITY_THRESHOLD, STABILITY_WINDOW)
        self.drift_monitor = BaselineDriftMonitor(BASELINE_DRIFT_WINDOW)

        self.weight_history = []
        self.weight_mean = 0.0
        self.weight_std = 0.5

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

    def read_device_weight_no_sw_tare(self):
        return struct.unpack("<f", self._readfrom_mem(self.REG_WEIGHT, 4))[0]

    def read_weight_raw(self):
        return self.read_device_weight_no_sw_tare() - self._tare_offset

    def reset_filters(self, keep_drift=False):
        self.weight_history = []
        self.weight_mean = 0.0
        self.weight_std = 0.5
        self.median_filter.clear()
        self.stability.clear()
        if not keep_drift:
            self.drift_monitor.clear()

    def read_weight(self):
        raw = self.read_weight_raw()

        if len(self.weight_history) < 3:
            self.weight_history.append(raw)
            self.median_filter.add(raw)
            self.stability.add(raw)
            if len(self.weight_history) >= 2:
                self.weight_mean = sum(self.weight_history) / len(self.weight_history)
            return raw

        self.median_filter.add(raw)
        filtered = self.median_filter.get()

        if len(self.weight_history) >= 20:
            z_score = abs(filtered - self.weight_mean) / max(self.weight_std, 0.5)
            if z_score > OUTLIER_REJECT_SIGMA:
                filtered = 0.7 * self.weight_mean + 0.3 * filtered

        self.weight_history.append(filtered)
        if len(self.weight_history) > 100:
            self.weight_history.pop(0)

        if len(self.weight_history) >= 5:
            self.weight_mean = sum(self.weight_history) / len(self.weight_history)
            if len(self.weight_history) >= 10:
                variance = sum((x - self.weight_mean) ** 2 for x in self.weight_history) / len(self.weight_history)
                self.weight_std = math.sqrt(variance) if variance > 0 else 0.1

        self.stability.add(filtered)
        self.drift_monitor.add_reading(filtered)
        return filtered

    def read_button(self):
        return self._readfrom_mem(self.REG_BUTTON, 1)[0] == 0

    def read_snapshot(self):
        weight = self.read_weight()
        return {
            "weight": weight,
            "raw": self.read_weight_raw(),
            "adc": self.read_adc(),
            "button": self.read_button(),
            "stable": self.stability.is_stable(),
            "drift": self.drift_monitor.get_drift_estimate(),
            "std": self.weight_std,
        }

    def set_led_order(self, order):
        self.led_order = "GRB" if str(order).upper() == "GRB" else "RGB"

    def set_led(self, r, g, b):
        if self.led_order == "GRB":
            payload = bytes((g & 0xFF, r & 0xFF, b & 0xFF))
        else:
            payload = bytes((r & 0xFF, g & 0xFF, b & 0xFF))
        self._writeto_mem(self.REG_LED, payload)

    def reset_internal_offset(self):
        self._writeto_mem(self.REG_RESET, b"\x01")
        time.sleep(0.1)
        self.reset_filters(keep_drift=False)

    def tare(self):
        samples = []
        for _ in range(30):
            samples.append(self.read_device_weight_no_sw_tare())
            time.sleep(0.02)
        samples.sort()
        self._tare_offset = samples[len(samples) // 2]
        self.reset_filters(keep_drift=True)

    def clear_tare(self):
        self._tare_offset = 0.0

    def get_drift_estimate(self):
        return self.drift_monitor.get_drift_estimate()

    def set_low_pass_filter(self, enable):
        self._writeto_mem(self.REG_LPF, b"\x01" if enable else b"\x00")

    def get_low_pass_filter(self):
        return self._readfrom_mem(self.REG_LPF, 1) == b"\x01"

    def set_average_level(self, level):
        level = max(0, min(50, int(level)))
        self._writeto_mem(self.REG_AVG, struct.pack("b", level))

    def get_average_level(self):
        return struct.unpack("b", self._readfrom_mem(self.REG_AVG, 1))[0]

    def set_ema_alpha(self, alpha):
        alpha = max(0, min(99, int(alpha)))
        self._writeto_mem(self.REG_EMA, struct.pack("b", alpha))

    def get_ema_alpha(self):
        return struct.unpack("b", self._readfrom_mem(self.REG_EMA, 1))[0]

    def write_gap(self, gap):
        self._writeto_mem(self.REG_GAP, struct.pack("<f", float(gap)))

    def calibrate_2point(self, w1_g, adc1, w2_g, adc2):
        if abs(w2_g - w1_g) < 1e-9:
            raise ValueError("Il faut deux masses différentes")
        gap = (adc2 - adc1) / float(w2_g - w1_g)
        self.write_gap(gap)
        return gap

display = board.DISPLAY
group = displayio.Group()
display.auto_refresh = True

def mk_label(text, x, y, scale=1, color=0xFFFFFF):
    return label.Label(terminalio.FONT, text=text, x=x, y=y, scale=scale, color=color)

try:
    display.root_group = None
except Exception:
    pass

try:
    background = displayio.Bitmap(display.width, display.height, 1)
    palette = displayio.Palette(1)
    palette[0] = COLOR_BG
    bg_sprite = displayio.TileGrid(background, pixel_shader=palette, x=0, y=0)
    group.append(bg_sprite)
except Exception:
    pass

#Affichage ecran et création des labels pour afficher les différentes informations sur les balances, leur état, les instructions, etc.

title_lbl = mk_label("STAGE LEO", 8, 16, scale=2, color=COLOR_TITLE)
mode_lbl = mk_label("OVERVIEW", 210, 16, scale=1, color=COLOR_ACCENT)
section_lbl = mk_label("BALANCE ACTIVE", 8, 38, scale=1, color=COLOR_DIM)
focus_lbl = mk_label("S-  ----.-- g", 8, 74, scale=3, color=COLOR_TEXT)
state_lbl = mk_label("", 8, 100, scale=1, color=COLOR_OK)
adc_lbl = mk_label("ADC: -----", 8, 122, scale=1, color=COLOR_SUB)
raw_lbl = mk_label("RAW: -----", 155, 122, scale=1, color=COLOR_SUB)
info_lbl = mk_label("", 8, 146, scale=1, color=COLOR_OK)
stats_lbl = mk_label("", 8, 164, scale=1, color=COLOR_SUB)
list_title_lbl = mk_label("VUE 5 BALANCES", 8, 186, scale=1, color=COLOR_DIM)
list_lbl = mk_label("", 8, 204, scale=1, color=COLOR_TEXT)
status_lbl = mk_label("", 8, 222, scale=1, color=COLOR_WARN)
hint_lbl = mk_label("L/R sel  A tare  B calib  C set  Joy log", 8, 238, scale=1, color=COLOR_ACCENT)

for widget in (
    title_lbl, mode_lbl, section_lbl, focus_lbl, state_lbl, adc_lbl, raw_lbl,
    info_lbl, stats_lbl, list_title_lbl, list_lbl, status_lbl, hint_lbl
):
    group.append(widget)

display.root_group = group

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
    time.sleep(0.08)

def probe_miniscale_on_current_channel(addr=SCALE_ADDR):
    ok = False
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
                ok = True
                break
            finally:
                root_i2c.unlock()
        except Exception:
            time.sleep(0.04)
    return ok

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

class CsvLogger:
    def __init__(self):
        self.enabled = False
        self.file = None
        self.last_log = 0.0

    def sd_mount(self):
        if not SD_AVAILABLE:
            raise RuntimeError("sdcardio/storage indisponibles")
        spi = busio.SPI(board.SD_SCK, board.SD_MOSI, board.SD_MISO)
        cs = DigitalInOut(board.SD_CS)
        cs.direction = Direction.OUTPUT
        cs.value = True
        sd = sdcardio.SDCard(spi, cs)
        vfs = storage.VfsFat(sd)
        storage.mount(vfs, "/sd")

    def open(self):
        self.sd_mount()
        try:
            self.file = open(LOG_PATH, "a")
            if self.file.tell() == 0:
                self.file.write("t_s,ch,weight_g,adc,stable,drift_est_g,std_g\n")
        except OSError:
            self.file = open(LOG_PATH, "w")
            self.file.write("t_s,ch,weight_g,adc,stable,drift_est_g,std_g\n")
        self.enabled = True

    def close(self):
        if self.file:
            try:
                self.file.flush()
                self.file.close()
            except Exception:
                pass
        self.file = None
        self.enabled = False

    def write_snapshots(self, now_s, scale_entries, snapshots):
        if not self.enabled or not self.file:
            return
        if (now_s - self.last_log) < LOG_PERIOD:
            return

        for entry in scale_entries:
            ch = entry["ch"]
            snap = snapshots.get(ch)
            if not snap or "error" in snap:
                continue
            self.file.write(
                "{:.3f},{},{:.3f},{},{},{:.3f},{:.3f}\n".format(
                    now_s, ch, snap["weight"], snap["adc"],
                    1 if snap["stable"] else 0, snap["drift"], snap["std"],
                )
            )
        self.file.flush()
        self.last_log = now_s

class ScaleApp:
    def __init__(self):
        self.mode = MODE_OVERVIEW
        self.scales = []
        self.active_idx = -1
        self.logger = CsvLogger()
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
                sc = MiniScaleEnhanced(mux_bus, name="S{}".format(ch), led_order="RGB")
                sc.set_ema_alpha(DEFAULT_ALPHA)
                sc.set_average_level(DEFAULT_AVG)
                sc.set_low_pass_filter(DEFAULT_LPF)
                sc.set_led(*LED_CAL_NO)
                new_scales.append({"ch": ch, "scale": sc, "led_order": "RGB", "calibrated": False})
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

    def update_display(self, now, snapshots):
        entry = self.active_entry()
        mode_lbl.text = MODE_NAMES.get(self.mode, "?")

        if not entry:
            focus_lbl.text = "AUCUNE BALANCE"
            focus_lbl.color = COLOR_ERR
            state_lbl.text = "Verifier TCA9548A / alim / cablage I2C"
            state_lbl.color = COLOR_WARN
            adc_lbl.text = "ADC: -----"
            raw_lbl.text = "RAW: -----"
            info_lbl.text = ""
            stats_lbl.text = ""
            list_lbl.text = ""
            return

        ch = entry["ch"]
        sc = entry["scale"]
        snap = snapshots.get(ch)

        title_lbl.text = "STAGE LEO  x{}  LOG {}".format(
            len(self.scales),
            "ON" if self.logger.enabled else "OFF"
        )

        if not snap or "error" in snap:
            focus_lbl.text = "S{}   ERREUR".format(ch)
            focus_lbl.color = COLOR_ERR
            state_lbl.text = "Erreur I2C"
            state_lbl.color = COLOR_ERR
            adc_lbl.text = "ADC: -----"
            raw_lbl.text = "RAW: -----"
            info_lbl.text = ""
            stats_lbl.text = ""
        else:
            focus_lbl.text = "S{} {:7.2f} g".format(ch, snap["weight"])
            focus_lbl.color = COLOR_TEXT

            if snap["stable"]:
                state_lbl.text = "STABLE"
                state_lbl.color = COLOR_OK
            else:
                state_lbl.text = "MESURE EN COURS"
                state_lbl.color = COLOR_WARN

            adc_lbl.text = "ADC: {}".format(snap["adc"])
            raw_lbl.text = "RAW: {:.2f} g".format(snap["raw"])
            info_lbl.text = "EMA={}  AVG={}  LPF={}  Drift:{:.2f} g".format(
                sc.get_ema_alpha(), sc.get_average_level(),
                "ON" if sc.get_low_pass_filter() else "OFF", snap["drift"],
            )
            stats_lbl.text = "CAL={}   σ={:.3f} g   BTN={}".format(
                "OK" if entry["calibrated"] else "NO",
                snap["std"],
                "ON" if snap["button"] else "--"
            )

        parts = []
        for e in self.scales:
            ch_i = e["ch"]
            snap_i = snapshots.get(ch_i)
            if not snap_i or "error" in snap_i:
                parts.append("S{}:Err".format(ch_i))
            else:
                parts.append("S{}:{:.1f}{}".format(ch_i, snap_i["weight"], "+" if snap_i["stable"] else ""))
        list_lbl.text = "  ".join(parts)

    def handle_navigation(self):
        if DB_L.update(pressed(JOY_LEFT)):
            self.next_active(-1)
        if DB_R.update(pressed(JOY_RIGHT)):
            self.next_active(+1)

    def toggle_logging(self):
        if not self.logger.enabled:
            try:
                self.logger.open()
                self.set_status("Logging ON")
            except Exception as e:
                self.set_status("Log err: {}".format(e))
        else:
            self.logger.close()
            self.set_status("Logging OFF")

    def handle_main_shortcuts(self):
        if DB_P.update(pressed(JOY_PRESS)):
            self.toggle_logging()

        if DB_A.update(pressed(BTN_A)):
            sc = self.active_scale()
            entry = self.active_entry()
            if sc and entry:
                self.set_status("[S{}] Tare...".format(entry["ch"]))
                sc.tare()
                self.set_status("[S{}] Tare OK".format(entry["ch"]))

        if DB_B.update(pressed(BTN_B)):
            sc = self.active_scale()
            entry = self.active_entry()
            if sc and entry:
                self.mode = MODE_CALIBRATION
                self.calibration_menu(sc, entry)
                self.mode = MODE_FOCUS

        if DB_C.update(pressed(BTN_C)):
            sc = self.active_scale()
            entry = self.active_entry()
            if sc and entry:
                self.mode = MODE_SETTINGS_SCALE
                self.settings_scale_menu(sc, entry)
                self.mode = MODE_FOCUS

    def handle_long_presses(self):
        if pressed(BTN_A):
            t0 = time.monotonic()
            while pressed(BTN_A):
                if time.monotonic() - t0 > 1.0:
                    self.mode = MODE_SETTINGS_ALL
                    self.settings_all_menu()
                    self.mode = MODE_FOCUS
                    while pressed(BTN_A):
                        time.sleep(0.05)
                    break
                time.sleep(0.05)

        if pressed(BTN_C):
            t0 = time.monotonic()
            while pressed(BTN_C):
                if time.monotonic() - t0 > 1.5:
                    self.set_status("Rescan TCA...")
                    self.init_scales()
                    while pressed(BTN_C):
                        time.sleep(0.05)
                    break
                time.sleep(0.05)

    def avg_adc_samples(self, scale, prefix=""):
        mean = 0.0
        M2 = 0.0
        step = max(1, SAMPLES_PER_POINT // 10)

        for k in range(SAMPLES_PER_POINT):
            x = float(scale.read_adc())
            delta = x - mean
            mean += delta / (k + 1)
            M2 += delta * (x - mean)

            if k % step == 0:
                sigma = math.sqrt(M2 / k) if k >= 1 else 0.0
                self.set_status("{} {}% σ~{:.0f}".format(
                    prefix, int(100 * k / SAMPLES_PER_POINT), sigma
                ))
            time.sleep(SAMPLE_DELAY)

        sigma_final = math.sqrt(M2 / (SAMPLES_PER_POINT - 1)) if SAMPLES_PER_POINT > 1 else 0.0
        return int(round(mean)), sigma_final

    def calibration_menu(self, scale, entry):
        ch = entry["ch"]
        items = ["2 points (0/100g)", "3 points (0/100/1000g)", "Annuler"]
        sel = 0

        while True:
            self.set_status("[S{}] {}".format(ch, items[sel]))

            if DB_U.update(pressed(JOY_UP)):
                sel = (sel - 1) % len(items)
            if DB_D.update(pressed(JOY_DOWN)):
                sel = (sel + 1) % len(items)

            if DB_B.update(pressed(BTN_B)):
                if sel == 0:
                    self.calibrate_2pt(scale, entry, 100.0)
                elif sel == 1:
                    self.calibrate_3pt(scale, entry)
                return

            if DB_C.update(pressed(BTN_C)):
                return
            time.sleep(0.05)

    def calibrate_2pt(self, scale, entry, ref_mass_g=100.0):
        ch = entry["ch"]
        self.set_status("[S{}] Reset offset...".format(ch))
        scale.reset_internal_offset()
        time.sleep(0.8)

        self.set_status("[S{}] Retirer poids".format(ch))
        time.sleep(1.0)
        adc0, _ = self.avg_adc_samples(scale, "[S{}] 0g".format(ch))

        self.set_status("[S{}] Poser {}g puis B".format(ch, int(ref_mass_g)))
        while True:
            if DB_B.update(pressed(BTN_B)):
                break
            time.sleep(0.05)

        adc1, _ = self.avg_adc_samples(scale, "[S{}] {}g".format(ch, int(ref_mass_g)))
        gap = scale.calibrate_2point(0.0, adc0, ref_mass_g, adc1)

        entry["calibrated"] = True
        scale.set_led(*LED_CAL_OK)
        self.set_status("[S{}] GAP={:.5f}".format(ch, gap))
        time.sleep(1.0)

    def calibrate_3pt(self, scale, entry):
        ch = entry["ch"]
        w1 = 100.0
        w2 = 1000.0

        self.set_status("[S{}] Reset offset...".format(ch))
        scale.reset_internal_offset()
        time.sleep(0.8)

        self.set_status("[S{}] Retirer poids".format(ch))
        time.sleep(1.0)
        adc0, _ = self.avg_adc_samples(scale, "[S{}] 0g".format(ch))

        self.set_status("[S{}] Poser {}g puis B".format(ch, int(w1)))
        while True:
            if DB_B.update(pressed(BTN_B)):
                break
            time.sleep(0.05)
        adc1, _ = self.avg_adc_samples(scale, "[S{}] {}g".format(ch, int(w1)))

        self.set_status("[S{}] Poser {}g puis B".format(ch, int(w2)))
        while True:
            if DB_B.update(pressed(BTN_B)):
                break
            time.sleep(0.05)
        adc2, _ = self.avg_adc_samples(scale, "[S{}] {}g".format(ch, int(w2)))

        num = w1 * (adc1 - adc0) + w2 * (adc2 - adc0)
        den = (w1 * w1 + w2 * w2)
        gap = num / den if den > 0 else 0.0
        scale.write_gap(gap)

        entry["calibrated"] = True
        scale.set_led(*LED_CAL_OK)
        self.set_status("[S{}] GAP={:.5f}".format(ch, gap))
        time.sleep(1.0)

    def settings_scale_menu(self, scale, entry):
        alpha = scale.get_ema_alpha()
        avg = scale.get_average_level()
        lpf = scale.get_low_pass_filter()
        led_order = entry["led_order"]

        items = [
            "EMA alpha", "AVG level", "LPF on/off", "LED RGB/GRB",
            "LED test", "Clear tare", "Reset filtres", "Quitter",
        ]
        sel = 0

        while True:
            self.set_status("[S{}] {}".format(entry["ch"], items[sel]))
            info_lbl.text = "EMA={} AVG={} LPF={} CAL={}".format(
                alpha, avg, "ON" if lpf else "OFF",
                "OK" if entry["calibrated"] else "NO",
            )

            if DB_U.update(pressed(JOY_UP)):
                sel = (sel - 1) % len(items)
            if DB_D.update(pressed(JOY_DOWN)):
                sel = (sel + 1) % len(items)

            if sel == 0:
                if DB_L.update(pressed(JOY_LEFT)):
                    alpha = max(0, alpha - 1)
                    scale.set_ema_alpha(alpha)
                if DB_R.update(pressed(JOY_RIGHT)):
                    alpha = min(99, alpha + 1)
                    scale.set_ema_alpha(alpha)
            elif sel == 1:
                if DB_L.update(pressed(JOY_LEFT)):
                    avg = max(0, avg - 1)
                    scale.set_average_level(avg)
                if DB_R.update(pressed(JOY_RIGHT)):
                    avg = min(50, avg + 1)
                    scale.set_average_level(avg)
            elif sel == 2:
                if DB_B.update(pressed(BTN_B)):
                    lpf = not lpf
                    scale.set_low_pass_filter(lpf)
            elif sel == 3:
                if DB_B.update(pressed(BTN_B)):
                    led_order = "GRB" if led_order == "RGB" else "RGB"
                    scale.set_led_order(led_order)
                    entry["led_order"] = led_order
            elif sel == 4:
                if DB_B.update(pressed(BTN_B)):
                    for color in LED_TEST_SEQUENCE:
                        scale.set_led(*color)
                        time.sleep(0.15)
                    scale.set_led(*(LED_CAL_OK if entry["calibrated"] else LED_CAL_NO))
            elif sel == 5:
                if DB_B.update(pressed(BTN_B)):
                    scale.clear_tare()
                    self.set_status("Tare effacee")
                    time.sleep(0.5)
            elif sel == 6:
                if DB_B.update(pressed(BTN_B)):
                    scale.reset_filters(keep_drift=False)
                    self.set_status("Filtres reset")
                    time.sleep(0.5)
            elif sel == 7:
                if DB_B.update(pressed(BTN_B)) or DB_C.update(pressed(BTN_C)):
                    return

            if DB_C.update(pressed(BTN_C)):
                return
            time.sleep(0.05)

    def apply_all(self, alpha=None, avg=None, lpf=None, led_order=None, clear_tare=False, reset_filters=False):
        for entry in self.scales:
            sc = entry["scale"]
            if alpha is not None:
                sc.set_ema_alpha(alpha)
            if avg is not None:
                sc.set_average_level(avg)
            if lpf is not None:
                sc.set_low_pass_filter(lpf)
            if led_order is not None:
                sc.set_led_order(led_order)
                entry["led_order"] = led_order
            if clear_tare:
                sc.clear_tare()
            if reset_filters:
                sc.reset_filters(keep_drift=False)

    def settings_all_menu(self):
        entry = self.active_entry()
        if entry:
            ref = entry["scale"]
            alpha = ref.get_ema_alpha()
            avg = ref.get_average_level()
            lpf = ref.get_low_pass_filter()
            led_order = entry["led_order"]
        else:
            alpha = DEFAULT_ALPHA
            avg = DEFAULT_AVG
            lpf = DEFAULT_LPF
            led_order = "RGB"

        items = [
            "EMA all", "AVG all", "LPF all", "LED order all",
            "LED test all", "Copy active -> all", "Clear tare all",
            "Reset filtres all", "Quitter",
        ]
        sel = 0

        while True:
            self.set_status("[ALL] {}".format(items[sel]))
            info_lbl.text = "EMA={} AVG={} LPF={} LED={}".format(
                alpha, avg, "ON" if lpf else "OFF", led_order,
            )

            if DB_U.update(pressed(JOY_UP)):
                sel = (sel - 1) % len(items)
            if DB_D.update(pressed(JOY_DOWN)):
                sel = (sel + 1) % len(items)

            if sel == 0:
                if DB_L.update(pressed(JOY_LEFT)):
                    alpha = max(0, alpha - 1)
                    self.apply_all(alpha=alpha)
                if DB_R.update(pressed(JOY_RIGHT)):
                    alpha = min(99, alpha + 1)
                    self.apply_all(alpha=alpha)
            elif sel == 1:
                if DB_L.update(pressed(JOY_LEFT)):
                    avg = max(0, avg - 1)
                    self.apply_all(avg=avg)
                if DB_R.update(pressed(JOY_RIGHT)):
                    avg = min(50, avg + 1)
                    self.apply_all(avg=avg)
            elif sel == 2:
                if DB_B.update(pressed(BTN_B)):
                    lpf = not lpf
                    self.apply_all(lpf=lpf)
            elif sel == 3:
                if DB_B.update(pressed(BTN_B)):
                    led_order = "GRB" if led_order == "RGB" else "RGB"
                    self.apply_all(led_order=led_order)
            elif sel == 4:
                if DB_B.update(pressed(BTN_B)):
                    for color in LED_TEST_SEQUENCE:
                        for e in self.scales:
                            e["scale"].set_led(*color)
                        time.sleep(0.15)
                    for e in self.scales:
                        e["scale"].set_led(*(LED_CAL_OK if e["calibrated"] else LED_CAL_NO))
            elif sel == 5:
                if DB_B.update(pressed(BTN_B)):
                    active = self.active_entry()
                    if active:
                        s = active["scale"]
                        self.apply_all(
                            alpha=s.get_ema_alpha(),
                            avg=s.get_average_level(),
                            lpf=s.get_low_pass_filter(),
                            led_order=active["led_order"],
                        )
                        self.set_status("Copie active -> all")
                        time.sleep(0.5)
            elif sel == 6:
                if DB_B.update(pressed(BTN_B)):
                    self.apply_all(clear_tare=True)
                    self.set_status("Tare all effacee")
                    time.sleep(0.5)
            elif sel == 7:
                if DB_B.update(pressed(BTN_B)):
                    self.apply_all(reset_filters=True)
                    self.set_status("Filtres all reset")
                    time.sleep(0.5)
            elif sel == 8:
                if DB_B.update(pressed(BTN_B)) or DB_C.update(pressed(BTN_C)):
                    return

            if DB_C.update(pressed(BTN_C)):
                return
            time.sleep(0.05)

    def loop(self):
        while True:
            now = time.monotonic()
            self.handle_navigation()
            self.handle_main_shortcuts()
            self.handle_long_presses()
            snapshots = self.build_snapshots()
            self.update_display(now, snapshots)
            self.logger.write_snapshots(now, self.scales, snapshots)
            time.sleep(0.03)

app = ScaleApp()
time.sleep(1.0)
app.loop()
