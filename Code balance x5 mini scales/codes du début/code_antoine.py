# Wio Terminal + (jusqu'à) 5x M5Stack Unit Mini Scales via TCA9548A
# VERSION ANTI-DRIFT pour surveillance évaporation long terme (plusieurs jours)
# Corrections: boutons fonctionnels, affichage poids, tare opérationnel

import time
import math
import struct
import board
import busio
from digitalio import DigitalInOut, Direction, Pull
import displayio
import terminalio
from adafruit_display_text import label

# --- SD card
try:
    import sdcardio, storage
    SD_AVAILABLE = True
except Exception:
    SD_AVAILABLE = False

# =========================
#  PARAMÈTRES ANTI-DRIFT (ajustés pour surveillance continue)
# =========================
AUTO_ZERO_ENABLED = False      # DÉSACTIVÉ car poids constant (évaporation)
BASELINE_DRIFT_WINDOW = 100    # fenêtre détection dérive
BASELINE_DRIFT_THRESHOLD = 10.0 # grammes - correction si dérive>10g
OUTLIER_REJECT_SIGMA = 4.0     # rejeter valeurs > 4σ (plus tolérant)
MEDIAN_FILTER_SIZE = 5         # filtre médian
STABILITY_WINDOW = 20          # échantillons pour stabilité
STABILITY_THRESHOLD = 0.2      # g - variation max pour "stable"

# =========================
#  Boutons / Joystick
# =========================
def _mk_button(pin):
    b = DigitalInOut(pin)
    b.direction = Direction.INPUT
    b.pull = Pull.UP
    return b

BTN_A = _mk_button(board.BUTTON_1)
BTN_B = _mk_button(board.BUTTON_2)
BTN_C = _mk_button(board.BUTTON_3)
JOY_UP = _mk_button(board.SWITCH_UP)
JOY_DOWN = _mk_button(board.SWITCH_DOWN)
JOY_LEFT = _mk_button(board.SWITCH_LEFT)
JOY_RIGHT = _mk_button(board.SWITCH_RIGHT)
JOY_PRESS = _mk_button(board.SWITCH_PRESS)

def pressed(btn): return not btn.value

class Debouncer:
    def __init__(self, delay=0.15):  # délai anti-rebond augmenté
        self._last = False
        self._last_change = 0
        self._delay = delay
    
    def update(self, val: bool) -> bool:
        now = time.monotonic()
        if val != self._last:
            if now - self._last_change > self._delay:
                self._last = val
                self._last_change = now
                return val  # front montant
        return False

dbA, dbB, dbC = Debouncer(), Debouncer(), Debouncer()
dbU, dbD, dbL, dbR, dbP = Debouncer(), Debouncer(), Debouncer(), Debouncer(), Debouncer()

# =========================
#  Filtre médian
# =========================
class MedianFilter:
    def __init__(self, size=5):
        self.size = size
        self.buffer = []
    
    def add(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.size:
            self.buffer.pop(0)
    
    def get(self):
        if not self.buffer:
            return 0.0
        sorted_buf = sorted(self.buffer)
        mid = len(sorted_buf) // 2
        if len(sorted_buf) % 2 == 0:
            return (sorted_buf[mid-1] + sorted_buf[mid]) / 2.0
        return sorted_buf[mid]
    
    def clear(self):
        self.buffer = []

# =========================
#  Détecteur stabilité
# =========================
class StabilityDetector:
    def __init__(self, threshold=0.2, required_samples=20):
        self.threshold = threshold
        self.required = required_samples
        self.buffer = []
    
    def add(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.required:
            self.buffer.pop(0)
    
    def is_stable(self):
        if len(self.buffer) < self.required:
            return False
        mean = sum(self.buffer) / len(self.buffer)
        max_dev = max(abs(v - mean) for v in self.buffer)
        return max_dev < self.threshold
    
    def get_stable_value(self):
        if self.is_stable() and self.buffer:
            return sum(self.buffer) / len(self.buffer)
        return None
    
    def clear(self):
        self.buffer = []

# =========================
#  Surveillance dérive baseline (pour correction manuelle)
# =========================
class BaselineDriftMonitor:
    def __init__(self, window=100):
        self.window = window
        self.history = []
        self.drift_detected = False
        self.estimated_drift = 0.0
    
    def add_reading(self, value):
        """Ajouter lecture pour surveillance tendance"""
        self.history.append(value)
        if len(self.history) > self.window:
            self.history.pop(0)
        
        # Calcul tendance linéaire simple
        if len(self.history) >= 20:
            recent = self.history[-20:]
            old = self.history[:20] if len(self.history) >= 40 else self.history[:10]
            avg_recent = sum(recent) / len(recent)
            avg_old = sum(old) / len(old)
            self.estimated_drift = avg_recent - avg_old
    
    def get_drift_estimate(self):
        return self.estimated_drift
    
    def clear(self):
        self.history = []
        self.drift_detected = False
        self.estimated_drift = 0.0

# =========================
#  TCA9548A wrapper
# =========================
class MuxChannel:
    def __init__(self, i2c: busio.I2C, tca_addr: int = 0x70, channel: int = 0):
        self._i2c = i2c
        self._tca = tca_addr
        if not (0 <= channel <= 7):
            raise ValueError("Canal TCA doit être 0..7")
        self._ch = channel

    def _select(self):
        got = self._i2c.try_lock()
        try:
            self._i2c.writeto(self._tca, bytes((1 << self._ch,)))
        finally:
            if got:
                self._i2c.unlock()

    def try_lock(self):
        ok = self._i2c.try_lock()
        if ok:
            self._i2c.writeto(self._tca, bytes((1 << self._ch,)))
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

# =========================
#  Driver MiniScale ANTI-DRIFT
# =========================
class MiniScaleEnhanced:
    DEFAULT_ADDR = 0x26
    REG_ADC   = 0x00
    REG_WEIGHT= 0x10
    REG_BUTTON= 0x20
    REG_LED   = 0x30
    REG_GAP   = 0x40
    REG_RESET = 0x50
    REG_LPF   = 0x80
    REG_AVG   = 0x81
    REG_EMA   = 0x82

    def __init__(self, i2c, address: int = DEFAULT_ADDR, name: str = "", led_order: str = "RGB"):
        addrs = i2c.scan()
        if address not in addrs:
            raise RuntimeError("MiniScale (0x%02X) introuvable" % address)
        self.i2c = i2c
        self.addr = address
        self._tare_offset = 0.0
        self.name = name or "Scale"
        self.led_order = led_order
        
        # Filtres anti-drift
        self.median_filter = MedianFilter(MEDIAN_FILTER_SIZE)
        self.stability = StabilityDetector(STABILITY_THRESHOLD, STABILITY_WINDOW)
        self.drift_monitor = BaselineDriftMonitor(BASELINE_DRIFT_WINDOW)
        
        # Tracking statistique pour outliers
        self.weight_history = []
        self.weight_mean = 0.0
        self.weight_std = 1.0
        
        # Compteur lectures pour debug
        self.read_count = 0

    def _writeto_mem(self, reg: int, data: bytes):
        while not self.i2c.try_lock(): pass
        try:
            self.i2c.writeto(self.addr, bytes((reg,)) + data)
        finally:
            self.i2c.unlock()

    def _readfrom_mem(self, reg: int, n: int) -> bytes:
        while not self.i2c.try_lock(): pass
        try:
            try:
                self.i2c.writeto(self.addr, bytes((reg,)), stop=False)
            except TypeError:
                self.i2c.writeto(self.addr, bytes((reg,)))
            buf = bytearray(n)
            self.i2c.readfrom_into(self.addr, buf)
            return bytes(buf)
        finally:
            self.i2c.unlock()

    def read_adc(self) -> int:
        return struct.unpack("<I", self._readfrom_mem(self.REG_ADC, 4))[0]

    def read_device_weight_no_sw_tare(self) -> float:
        return struct.unpack("<f", self._readfrom_mem(self.REG_WEIGHT, 4))[0]

    def read_weight_raw(self) -> float:
        """Lecture brute avec tare seulement"""
        return self.read_device_weight_no_sw_tare() - self._tare_offset

    def read_weight(self) -> float:
        """Lecture avec filtres anti-drift"""
        self.read_count += 1
        raw = self.read_weight_raw()
        
        # Si pas assez d'historique, retourner valeur brute
        if len(self.weight_history) < 3:
            self.weight_history.append(raw)
            self.median_filter.add(raw)
            self.stability.add(raw)
            if len(self.weight_history) >= 2:
                self.weight_mean = sum(self.weight_history) / len(self.weight_history)
            return raw
        
        # 1. Filtre médian
        self.median_filter.add(raw)
        filtered = self.median_filter.get()
        
        # 2. Rejet outliers modéré (si historique suffisant)
        if len(self.weight_history) >= 20:
            z_score = abs(filtered - self.weight_mean) / max(self.weight_std, 0.5)
            if z_score > OUTLIER_REJECT_SIGMA:
                # Utiliser moyenne pondérée au lieu de rejeter complètement
                filtered = 0.7 * self.weight_mean + 0.3 * filtered
        
        # 3. Mise à jour statistiques
        self.weight_history.append(filtered)
        if len(self.weight_history) > 100:
            self.weight_history.pop(0)
        
        if len(self.weight_history) >= 5:
            self.weight_mean = sum(self.weight_history) / len(self.weight_history)
            if len(self.weight_history) >= 10:
                variance = sum((x - self.weight_mean)**2 for x in self.weight_history) / len(self.weight_history)
                self.weight_std = math.sqrt(variance) if variance > 0 else 0.1
        
        # 4. Détection stabilité
        self.stability.add(filtered)
        
        # 5. Surveillance dérive (pas de correction auto)
        self.drift_monitor.add_reading(filtered)
        
        return filtered

    def read_button(self) -> bool:
        return self._readfrom_mem(self.REG_BUTTON, 1)[0] == 0

    def set_led_order(self, order: str):
        self.led_order = "GRB" if str(order).upper() == "GRB" else "RGB"

    def set_led(self, r: int, g: int, b: int):
        if self.led_order == "GRB":
            payload = bytes((g & 0xFF, r & 0xFF, b & 0xFF))
        else:
            payload = bytes((r & 0xFF, g & 0xFF, b & 0xFF))
        self._writeto_mem(self.REG_LED, payload)

    def reset_internal_offset(self):
        self._writeto_mem(self.REG_RESET, b"\x01")
        time.sleep(0.1)
        # Réinitialiser filtres
        self.median_filter.clear()
        self.stability.clear()
        self.drift_monitor.clear()
        self.weight_history = []
        self.read_count = 0

    def tare(self):
        """Tare robuste avec moyenne de plusieurs lectures"""
        samples = []
        for _ in range(30):  # 30 lectures sur ~0.6s
            samples.append(self.read_device_weight_no_sw_tare())
            time.sleep(0.02)
        # Utiliser médiane pour robustesse
        samples.sort()
        self._tare_offset = samples[len(samples)//2]
        
        # NE PAS réinitialiser complètement - juste vider les buffers
        self.weight_history = []
        self.weight_mean = 0.0
        self.weight_std = 0.5
        self.median_filter.clear()
        self.stability.clear()
        # Garder drift_monitor pour continuité

    def clear_tare(self):
        self._tare_offset = 0.0

    def is_stable(self):
        return self.stability.is_stable()
    
    def get_stable_value(self):
        return self.stability.get_stable_value()
    
    def get_drift_estimate(self):
        """Estimation dérive détectée (g)"""
        return self.drift_monitor.get_drift_estimate()

    # Filtres hardware
    def set_low_pass_filter(self, enable: bool):
        self._writeto_mem(self.REG_LPF, b"\x01" if enable else b"\x00")

    def get_low_pass_filter(self) -> bool:
        return self._readfrom_mem(self.REG_LPF, 1) == b"\x01"

    def set_average_level(self, level: int):
        level = max(0, min(50, level))
        self._writeto_mem(self.REG_AVG, struct.pack("b", level))

    def get_average_level(self) -> int:
        return struct.unpack("b", self._readfrom_mem(self.REG_AVG, 1))[0]

    def set_ema_alpha(self, alpha: int):
        alpha = max(0, min(99, alpha))
        self._writeto_mem(self.REG_EMA, struct.pack("b", alpha))

    def get_ema_alpha(self) -> int:
        return struct.unpack("b", self._readfrom_mem(self.REG_EMA, 1))[0]

    def write_gap(self, gap: float):
        self._writeto_mem(self.REG_GAP, struct.pack("<f", gap))

    def calibrate_2point(self, w1_g: float, adc1: int, w2_g: float, adc2: int) -> float:
        if abs(w2_g - w1_g) < 1e-9:
            raise ValueError("Masses différentes requises")
        gap = (adc2 - adc1) / float(w2_g - w1_g)
        self.write_gap(gap)
        return gap

# =========================
#  Paramètres
# =========================
DEFAULT_ALPHA = 8      # Lissage EMA fort pour stabilité long terme
DEFAULT_AVG   = 50     # Moyenne hardware max
DEFAULT_LPF   = True   # LPF actif

SAMPLES_PER_POINT = 500
SPS_HINT = 10
SAMPLE_DELAY = 1.0 / SPS_HINT

# =========================
#  Affichage
# =========================
display = board.DISPLAY
display.auto_refresh = True
main_group = displayio.Group()

def mk_label(text, x, y, scale=2, color=0xFFFFFF):
    return label.Label(terminalio.FONT, text=text, color=color, x=x, y=y, scale=scale)

title = mk_label("Scales EVAPORATION x5", 6, 20, scale=2, color=0x00FFD0)
focus_lbl = mk_label("Ch:--  ----.-- g", 6, 58, scale=3, color=0xFFFFFF)
adc_lbl = mk_label("ADC: -----", 6, 98, scale=2, color=0xA0A0A0)
filt_lbl = mk_label("", 6, 124, scale=1, color=0x80FF80)
btn_lbl  = mk_label("", 6, 140, scale=1, color=0xFFFF80)
status_lbl = mk_label("", 6, 158, scale=1, color=0xFFB070)
list_lbl = mk_label("", 6, 188, scale=1, color=0xFFFFFF)
hint_lbl = mk_label("[L/R]=Switch [A]=Tare [B]=Calib [C]=Set [PRESS]=Log", 6, 226, scale=1, color=0x80B0FF)

for w in (title, focus_lbl, adc_lbl, filt_lbl, btn_lbl, status_lbl, list_lbl, hint_lbl):
    main_group.append(w)
display.root_group = main_group

# =========================
#  I2C + TCA
# =========================
root_i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

def detect_tca_addr(i2c):
    while not i2c.try_lock(): pass
    try:
        addrs = set(i2c.scan())
    finally:
        i2c.unlock()
    for a in range(0x70, 0x78):
        if a in addrs:
            return a
    raise RuntimeError("TCA9548A introuvable")

MUX_ADDR = detect_tca_addr(root_i2c)

def tca_write_mask(mask: int):
    while not root_i2c.try_lock(): pass
    try:
        root_i2c.writeto(MUX_ADDR, bytes((mask,)))
    finally:
        root_i2c.unlock()

def tca_select_channel(ch: int):
    tca_write_mask(1 << ch)
    time.sleep(0.08)

def probe_miniscale_on_current_channel(addr=0x26):
    ok = False
    for _ in range(3):
        try:
            while not root_i2c.try_lock(): pass
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

def detect_scale_channels(addr=0x26):
    found = []
    tca_write_mask(0x00)
    time.sleep(0.02)
    for ch in range(8):
        tca_select_channel(ch)
        if probe_miniscale_on_current_channel(addr):
            found.append(ch)
    tca_write_mask(0x00)
    return found

all_found = detect_scale_channels()
status_lbl.text = "Balances: " + ",".join(str(x) for x in all_found)
time.sleep(1.0)

TARGET_COUNT = 5
CHANNELS = all_found[:TARGET_COUNT] if len(all_found) >= TARGET_COUNT else all_found[:]

def set_led_calibrated(scale_obj, is_calibrated: bool):
    try:
        if is_calibrated:
            scale_obj.set_led(0, 60, 0)
        else:
            scale_obj.set_led(80, 0, 0)
    except Exception:
        pass

def rebuild_scales(CHANNELS):
    global scales, active_idx
    scales = []
    for ch in CHANNELS:
        mux_bus = MuxChannel(root_i2c, tca_addr=MUX_ADDR, channel=ch)
        try:
            s = MiniScaleEnhanced(mux_bus, name=f"S{ch}", led_order="RGB")
            s.set_ema_alpha(DEFAULT_ALPHA)
            s.set_average_level(DEFAULT_AVG)
            s.set_low_pass_filter(DEFAULT_LPF)
            set_led_calibrated(s, False)
            scales.append([ch, s, "RGB", False])
        except Exception as e:
            print(f"[rebuild] Canal {ch}: {e}")
    active_idx = 0 if scales else -1

rebuild_scales(CHANNELS)

def next_active(delta):
    global active_idx
    n = len(scales)
    if n == 0: 
        return
    active_idx = (active_idx + delta) % n

# =========================
#  Calibration
# =========================
def avg_adc_samples(scale: MiniScaleEnhanced, samples: int = SAMPLES_PER_POINT,
                    delay: float = SAMPLE_DELAY, progress_prefix: str = "",
                    gap_for_sigma=None):
    mean = 0.0
    M2 = 0.0
    step = max(1, samples // 10)
    for k in range(samples):
        x = float(scale.read_adc())
        delta = x - mean
        mean += delta / (k + 1)
        M2 += delta * (x - mean)
        if k % step == 0:
            sigma = math.sqrt(M2 / k) if k >= 1 else 0.0
            try:
                status_lbl.text = f"{progress_prefix} {int(100*k/samples)}%  σ≈{sigma:.0f}"
            except Exception:
                pass
        time.sleep(delay)
    sigma_final = math.sqrt(M2 / (samples - 1)) if samples > 1 else 0.0
    return int(round(mean)), sigma_final, 0.0

def calibration_sampling_menu():
    global SAMPLES_PER_POINT
    choices = [100, 300, 500, 1000]
    try:
        idx = choices.index(SAMPLES_PER_POINT)
    except ValueError:
        idx = 2
    status_lbl.text = "Echantillons: UP/DOWN, B=OK, C=annuler"
    
    while True:
        if dbU.update(pressed(JOY_UP)):   
            idx = (idx - 1) % len(choices)
        if dbD.update(pressed(JOY_DOWN)): 
            idx = (idx + 1) % len(choices)
        
        filt_lbl.text = f"Calib: [{choices[idx]}] ~{int(choices[idx]/SPS_HINT)}s/pt"
        
        if dbB.update(pressed(BTN_B)):
            SAMPLES_PER_POINT = choices[idx]
            status_lbl.text = f"OK: {SAMPLES_PER_POINT} ech/pt"
            time.sleep(0.3)
            return True
        if dbC.update(pressed(BTN_C)):
            status_lbl.text = "Annule"
            time.sleep(0.3)
            return False
        time.sleep(0.05)

def calibration_wizard_2pt(scale: MiniScaleEnhanced, ch: int, ref_mass_g=100.0):
    status_lbl.text = f"[S{ch}] 2pt: retirer poids, reset..."
    scale.reset_internal_offset()
    time.sleep(0.8)

    status_lbl.text = f"[S{ch}] 0g ({SAMPLES_PER_POINT})..."
    adc0, sigma0, _ = avg_adc_samples(scale, progress_prefix=f"[S{ch}] 0g")

    status_lbl.text = f"[S{ch}] Placer {int(ref_mass_g)}g puis B"
    while True:
        if dbB.update(pressed(BTN_B)): 
            break
        time.sleep(0.05)
    
    status_lbl.text = f"[S{ch}] {int(ref_mass_g)}g ({SAMPLES_PER_POINT})..."
    adc1, sigma1, _ = avg_adc_samples(scale, progress_prefix=f"[S{ch}] {int(ref_mass_g)}g")

    gap = scale.calibrate_2point(0.0, adc0, ref_mass_g, adc1)
    status_lbl.text = f"[S{ch}] OK GAP={gap:.5f}"
    time.sleep(1.0)
    
    for i,(ch_i, sc_i, lo_i, cal_i) in enumerate(scales):
        if ch_i == ch:
            scales[i][3] = True
            set_led_calibrated(scales[i][1], True)
            break
    return gap

def calibration_wizard_3pt(scale: MiniScaleEnhanced, ch: int):
    W1, W2 = 100.0, 1000.0

    status_lbl.text = f"[S{ch}] 3pt: reset..."
    scale.reset_internal_offset()
    time.sleep(0.8)

    status_lbl.text = f"[S{ch}] 0g..."
    adc0, _, _ = avg_adc_samples(scale, progress_prefix=f"[S{ch}] 0g")

    status_lbl.text = f"[S{ch}] {int(W1)}g puis B"
    while True:
        if dbB.update(pressed(BTN_B)): break
        time.sleep(0.05)
    adc1, _, _ = avg_adc_samples(scale, progress_prefix=f"[S{ch}] {int(W1)}g")

    status_lbl.text = f"[S{ch}] {int(W2)}g puis B"
    while True:
        if dbB.update(pressed(BTN_B)): break
        time.sleep(0.05)
    adc2, _, _ = avg_adc_samples(scale, progress_prefix=f"[S{ch}] {int(W2)}g")

    num = W1*(adc1 - adc0) + W2*(adc2 - adc0)
    den = (W1*W1 + W2*W2)
    gap = num / den if den > 0 else 0.0
    scale.write_gap(gap)

    status_lbl.text = f"[S{ch}] OK GAP={gap:.5f}"
    time.sleep(1.0)

    for i,(ch_i, sc_i, lo_i, cal_i) in enumerate(scales):
        if ch_i == ch:
            scales[i][3] = True
            set_led_calibrated(scales[i][1], True)
            break
    return gap

def choose_calibration_menu(scale: MiniScaleEnhanced, ch: int):
    if not calibration_sampling_menu():
        return False

    items = ["2pt (0/100g)", "3pt (0/100/1000g)", "Annuler"]
    sel = 0
    def render():
        status_lbl.text = f"[S{ch}] {items[sel]} (UP/DOWN, B=OK)"
    render()
    
    while True:
        if dbU.update(pressed(JOY_UP)):   
            sel = (sel - 1) % len(items)
            render()
        if dbD.update(pressed(JOY_DOWN)): 
            sel = (sel + 1) % len(items)
            render()
        if dbB.update(pressed(BTN_B)):
            break
        if dbC.update(pressed(BTN_C)):
            return False
        time.sleep(0.05)

    if sel == 0:
        calibration_wizard_2pt(scale, ch, ref_mass_g=100.0)
    elif sel == 1:
        calibration_wizard_3pt(scale, ch)
    else:
        return False
    return True

# =========================
#  Réglages
# =========================
def settings_screen(scale: MiniScaleEnhanced, ch: int, ref_led_order_container):
    alpha = scale.get_ema_alpha()
    avg = scale.get_average_level()
    lpf = scale.get_low_pass_filter()
    led_order = ref_led_order_container[0]
    items = ["EMA alpha", "AVG level", "LPF on/off", "LED order", "LED test", 
             "Clear tare", "Reset filtres", "Quitter"]
    sel = 0

    def render():
        cal_flag = "OK" if [t for t in scales if t[0]==ch][0][3] else "NO"
        drift = scale.get_drift_estimate()
        status_lbl.text = f"[S{ch}] {items[sel]} | Drift:{drift:.2f}g"
        filt_lbl.text = f"EMA={alpha} AVG={avg} LPF={'ON' if lpf else 'OFF'} CAL:{cal_flag}"

    render()
    while True:
        if dbU.update(pressed(JOY_UP)):   
            sel = (sel - 1) % len(items)
            render()
        if dbD.update(pressed(JOY_DOWN)): 
            sel = (sel + 1) % len(items)
            render()

        if sel == 0:
            if dbL.update(pressed(JOY_LEFT)):  
                alpha = max(0, alpha-1)
                scale.set_ema_alpha(alpha)
                render()
            if dbR.update(pressed(JOY_RIGHT)): 
                alpha = min(99, alpha+1)
                scale.set_ema_alpha(alpha)
                render()
        elif sel == 1:
            if dbL.update(pressed(JOY_LEFT)):  
                avg = max(0, avg-1)
                scale.set_average_level(avg)
                render()
            if dbR.update(pressed(JOY_RIGHT)): 
                avg = min(50, avg+1)
                scale.set_average_level(avg)
                render()
        elif sel == 2:
            if dbB.update(pressed(BTN_B)): 
                lpf = not lpf
                scale.set_low_pass_filter(lpf)
                render()
        elif sel == 3:
            if dbB.update(pressed(BTN_B)):
                led_order = "GRB" if led_order == "RGB" else "RGB"
                scale.set_led_order(led_order)
                ref_led_order_container[0] = led_order
                render()
        elif sel == 4:
            if dbB.update(pressed(BTN_B)):
                for c in [(0,30,10),(20,0,80),(80,20,0),(0,0,0)]:
                    scale.set_led(*c)
                    time.sleep(0.15)
                render()
        elif sel == 5:
            if dbB.update(pressed(BTN_B)):
                scale.clear_tare()
                status_lbl.text="Tare effacee"
                time.sleep(0.5)
                render()
        elif sel == 6:
            if dbB.update(pressed(BTN_B)):
                scale.weight_history = []
                scale.median_filter.clear()
                scale.stability.clear()
                scale.drift_monitor.clear()
                status_lbl.text="Filtres reset"
                time.sleep(0.5)
                render()
        elif sel == 7:
            if dbB.update(pressed(BTN_B)) or dbC.update(pressed(BTN_C)): 
                break
        if dbC.update(pressed(BTN_C)): 
            break
        time.sleep(0.05)

def apply_all(alpha=None, avg=None, lpf=None, led_order=None, clear_tare=False, reset_filters=False):
    for i,(ch_i, sc_i, lo_i, cal_i) in enumerate(scales):
        if alpha is not None: sc_i.set_ema_alpha(alpha)
        if avg   is not None: sc_i.set_average_level(avg)
        if lpf   is not None: sc_i.set_low_pass_filter(lpf)
        if led_order is not None:
            sc_i.set_led_order(led_order)
            scales[i][2] = led_order
        if clear_tare: 
            sc_i.clear_tare()
        if reset_filters:
            sc_i.weight_history = []
            sc_i.median_filter.clear()
            sc_i.stability.clear()
            sc_i.drift_monitor.clear()

def global_settings_screen():
    if active_idx >= 0 and len(scales)>0:
        _, s0, lo0, _ = scales[active_idx]
        alpha = s0.get_ema_alpha()
        avg = s0.get_average_level()
        lpf = s0.get_low_pass_filter()
        led_order = lo0
    else:
        alpha, avg, lpf, led_order = DEFAULT_ALPHA, DEFAULT_AVG, DEFAULT_LPF, "RGB"

    items = ["EMA (ALL)", "AVG (ALL)", "LPF (ALL)", "LED order (ALL)", 
             "LED test ALL", "Copy ACTIVE->ALL", "Clear tare ALL", 
             "Reset filtres ALL", "Quitter"]
    sel = 0

    def render():
        status_lbl.text = f"[ALL] {items[sel]}"
        filt_lbl.text = f"EMA={alpha} AVG={avg} LPF={'ON' if lpf else 'OFF'} LED={led_order}"

    render()
    while True:
        if dbU.update(pressed(JOY_UP)):   
            sel = (sel - 1) % len(items)
            render()
        if dbD.update(pressed(JOY_DOWN)): 
            sel = (sel + 1) % len(items)
            render()

        if sel == 0:
            if dbL.update(pressed(JOY_LEFT)):  
                alpha = max(0, alpha-1)
                apply_all(alpha=alpha)
                render()
            if dbR.update(pressed(JOY_RIGHT)): 
                alpha = min(99, alpha+1)
                apply_all(alpha=alpha)
                render()
        elif sel == 1:
            if dbL.update(pressed(JOY_LEFT)):  
                avg = max(0, avg-1)
                apply_all(avg=avg)
                render()
            if dbR.update(pressed(JOY_RIGHT)): 
                avg = min(50, avg+1)
                apply_all(avg=avg)
                render()
        elif sel == 2:
            if dbB.update(pressed(BTN_B)): 
                lpf = not lpf
                apply_all(lpf=lpf)
                render()
        elif sel == 3:
            if dbB.update(pressed(BTN_B)):
                led_order = "GRB" if led_order == "RGB" else "RGB"
                apply_all(led_order=led_order)
                render()
        elif sel == 4:
            if dbB.update(pressed(BTN_B)):
                for c in [(0,30,10),(20,0,80),(80,20,0),(0,0,0)]:
                    for _, sc_i, _, _ in scales: 
                        sc_i.set_led(*c)
                    time.sleep(0.15)
                render()
        elif sel == 5:
            if dbB.update(pressed(BTN_B)) and active_idx>=0:
                _, sA, loA, _ = scales[active_idx]
                apply_all(alpha=sA.get_ema_alpha(),
                          avg=sA.get_average_level(),
                          lpf=sA.get_low_pass_filter(),
                          led_order=loA)
                status_lbl.text="Config copiee"
                time.sleep(0.5)
                render()
        elif sel == 6:
            if dbB.update(pressed(BTN_B)):
                apply_all(clear_tare=True)
                status_lbl.text="Tare ALL effacee"
                time.sleep(0.5)
                render()
        elif sel == 7:
            if dbB.update(pressed(BTN_B)):
                apply_all(reset_filters=True)
                status_lbl.text="Filtres ALL reset"
                time.sleep(0.5)
                render()
        elif sel == 8:
            if dbB.update(pressed(BTN_B)) or dbC.update(pressed(BTN_C)): 
                break
        if dbC.update(pressed(BTN_C)): 
            break
        time.sleep(0.05)

# =========================
#  Logger CSV
# =========================
LOG_ENABLED = False
LOG_FILE = None
LOG_PATH = "/sd/evaporation_log.csv"
LOG_PERIOD = 1.0  # 1 seconde par défaut
last_log = 0.0

def sd_mount():
    if not SD_AVAILABLE:
        raise RuntimeError("sdcardio/storage indisponibles")
    spi = busio.SPI(board.SD_SCK, board.SD_MOSI, board.SD_MISO)
    cs = DigitalInOut(board.SD_CS)
    cs.direction = Direction.OUTPUT
    cs.value = True
    sd = sdcardio.SDCard(spi, cs)
    vfs = storage.VfsFat(sd)
    storage.mount(vfs, "/sd")

def open_log():
    global LOG_FILE
    try:
        sd_mount()
    except Exception as e:
        status_lbl.text = f"SD err: {e}"
        return False
    try:
        try:
            LOG_FILE = open(LOG_PATH, "a")
            if LOG_FILE.tell() == 0:
                LOG_FILE.write("t_s,ch,weight_g,adc,stable,drift_est_g,std_g\n")
        except OSError:
            LOG_FILE = open(LOG_PATH, "w")
            LOG_FILE.write("t_s,ch,weight_g,adc,stable,drift_est_g,std_g\n")
        return True
    except Exception as e:
        status_lbl.text = f"Log err: {e}"
        return False

def close_log():
    global LOG_FILE
    if LOG_FILE:
        try:
            LOG_FILE.flush()
            LOG_FILE.close()
        except Exception:
            pass
        LOG_FILE = None

# =========================
#  Boucle principale
# =========================
last_ui = time.monotonic()
last_button_check = time.monotonic()

status_lbl.text = "Init OK. Pret."
time.sleep(1.0)

while True:
    now = time.monotonic()
    
    # Vérification boutons avec timing séparé
    if now - last_button_check > 0.05:
        # Long press A => GLOBAL settings
        if pressed(BTN_A):
            t0 = time.monotonic()
            while pressed(BTN_A):
                if time.monotonic() - t0 > 1.0:
                    global_settings_screen()
                    while pressed(BTN_A): 
                        time.sleep(0.05)
                    break
                time.sleep(0.05)
        
        # Navigation
        if dbL.update(pressed(JOY_LEFT)):  
            next_active(-1)
            status_lbl.text = f"Balance S{scales[active_idx][0]}" if scales else ""
        
        if dbR.update(pressed(JOY_RIGHT)): 
            next_active(+1)
            status_lbl.text = f"Balance S{scales[active_idx][0]}" if scales else ""
        
        # Toggle logging avec JOY_PRESS
        if dbP.update(pressed(JOY_PRESS)):
            if not LOG_ENABLED:
                if open_log():
                    LOG_ENABLED = True
                    status_lbl.text = "Logging ON -> SD"
                    time.sleep(0.5)
            else:
                LOG_ENABLED = False
                close_log()
                status_lbl.text = "Logging OFF"
                time.sleep(0.5)
        
        last_button_check = now
    
    # Lecture et affichage balance active
    if active_idx >= 0 and len(scales) > 0:
        ch, s, led_order, calibrated = scales[active_idx]
        
        try:
            w = s.read_weight()
            w_raw = s.read_weight_raw()  # Lecture brute pour debug
            a = s.read_adc()
            unit_btn = s.read_button()
            stable = s.is_stable()
            drift_est = s.get_drift_estimate()
        except Exception as e:
            w, w_raw, a, unit_btn, stable, drift_est = float("nan"), float("nan"), 0, False, False, 0.0
            status_lbl.text = f"[S{ch}] Err I2C: {e}"
        
        # Affichage principal avec indicateur stabilité
        stable_marker = " [OK]" if stable else ""
        focus_lbl.text = f"Ch:{ch}  {w:7.2f}g{stable_marker}"
        adc_lbl.text = f"ADC:{a} Raw:{w_raw:.2f}g"  # Afficher ADC ET poids brut
        btn_lbl.text = f"Btn[S{ch}]: {'PRESS' if unit_btn else '-'}"
        
        # Liste toutes balances
        vals = []
        for ch_i, sc_i, _, _ in scales:
            try:
                wi = sc_i.read_weight()
                si = "+" if sc_i.is_stable() else ""
                vals.append(f"S{ch_i}:{wi:.1f}{si}")
            except Exception:
                vals.append(f"S{ch_i}:Err")
        list_lbl.text = " ".join(vals)
        
        # Mise à jour périodique infos filtres
        if now - last_ui > 1.0:
            filt_lbl.text = (f"EMA={s.get_ema_alpha()} AVG={s.get_average_level()} "
                           f"LPF={'ON' if s.get_low_pass_filter() else 'OFF'} "
                           f"CAL:{'OK' if calibrated else 'NO'} "
                           f"Drift:{drift_est:.2f}g σ:{s.weight_std:.2f}g")
            title.text = f"EVAPORATION x{len(scales)} [S{ch}] Log:{'ON' if LOG_ENABLED else 'OFF'}"
            last_ui = now
        
        # Boutons actions balance active
        if dbA.update(pressed(BTN_A)):
            status_lbl.text = f"[S{ch}] Tare en cours..."
            s.tare()
            status_lbl.text = f"[S{ch}] Tare OK"
            time.sleep(0.5)
        
        if dbB.update(pressed(BTN_B)):
            choose_calibration_menu(s, ch)
            # MAJ état calibration
            for i,(ch_i, sc_i, lo_i, cal_i) in enumerate(scales):
                if ch_i == ch:
                    calibrated = scales[i][3]
                    break
        
        if dbC.update(pressed(BTN_C)):
            settings_screen(s, ch, ref_led_order_container=scales[active_idx][2:3])
    
    # Long press C => rescan
    if pressed(BTN_C):
        t0 = time.monotonic()
        while pressed(BTN_C):
            if time.monotonic() - t0 > 1.5:
                status_lbl.text = "Rescan TCA..."
                all_found = detect_scale_channels()
                CHANNELS = all_found[:5] if len(all_found) >= 5 else all_found[:]
                rebuild_scales(CHANNELS)
                status_lbl.text = f"Rescan OK: {len(scales)} balances"
                time.sleep(1.0)
                while pressed(BTN_C): 
                    time.sleep(0.05)
                break
            time.sleep(0.05)
    
    # Logging périodique
    if LOG_ENABLED and LOG_FILE and (now - last_log) >= LOG_PERIOD:
        try:
            for ch_i, sc_i, _, _ in scales:
                wi = sc_i.read_weight()
                ai = sc_i.read_adc()
                stable_flag = 1 if sc_i.is_stable() else 0
                drift_est = sc_i.get_drift_estimate()
                std = sc_i.weight_std
                LOG_FILE.write(f"{now:.3f},{ch_i},{wi:.3f},{ai},{stable_flag},{drift_est:.3f},{std:.3f}\n")
            LOG_FILE.flush()
            last_log = now
        except Exception as e:
            status_lbl.text = f"Log err: {e}"
            LOG_ENABLED = False
            close_log()
    
    # Cas sans balance
    if active_idx < 0 or len(scales) == 0:
        focus_lbl.text = "Ch:--  ----.-- g"
        list_lbl.text = "Aucune balance detectee"
        filt_lbl.text = "Appui long C = rescan"
        time.sleep(0.5)
    
    time.sleep(0.03)