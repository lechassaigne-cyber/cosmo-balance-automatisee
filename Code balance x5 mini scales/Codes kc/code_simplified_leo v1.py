# Wio Terminal + (jusqu'à) 5x M5Stack Unit Mini Scales via TCA9548A
# VERSION ANTI-DRIFT - SIMPLIFIÉE

import time, math, struct, board, busio
from digitalio import DigitalInOut, Direction, Pull
import displayio, terminalio
from adafruit_display_text import label

# SD Card optionnel
try:
    import sdcardio, storage
    SD_AVAILABLE = True
except:
    SD_AVAILABLE = False

# ============= PARAMÈTRES =============
AUTO_ZERO_ENABLED = False
BASELINE_DRIFT_WINDOW = 100
BASELINE_DRIFT_THRESHOLD = 10.0
OUTLIER_REJECT_SIGMA = 4.0
MEDIAN_FILTER_SIZE = 5
STABILITY_WINDOW = 20
STABILITY_THRESHOLD = 0.2
DEFAULT_ALPHA = 8
DEFAULT_AVG = 50
DEFAULT_LPF = True
SAMPLES_PER_POINT = 500
SPS_HINT = 10
SAMPLE_DELAY = 1.0 / SPS_HINT

# ============= BOUTONS =============
def _mk_button(pin):
    b = DigitalInOut(pin)
    b.direction = Direction.INPUT
    b.pull = Pull.UP
    return b

BTN_A = _mk_button(board.BUTTON_1)
BTN_B = _mk_button(board.BUTTON_2)
BTN_C = _mk_button(board.BUTTON_3)
JOY = {
    'UP': _mk_button(board.SWITCH_UP),
    'DOWN': _mk_button(board.SWITCH_DOWN),
    'LEFT': _mk_button(board.SWITCH_LEFT),
    'RIGHT': _mk_button(board.SWITCH_RIGHT),
    'PRESS': _mk_button(board.SWITCH_PRESS)
}

class Debouncer:
    def __init__(self, delay=0.15):
        self._last = False
        self._last_change = 0
        self._delay = delay
    
    def update(self, val: bool) -> bool:
        now = time.monotonic()
        if val != self._last and now - self._last_change > self._delay:
            self._last = val
            self._last_change = now
            return True
        return False

debouncers = {btn: Debouncer() for btn in ['A', 'B', 'C', 'UP', 'DOWN', 'LEFT', 'RIGHT', 'PRESS']}

def pressed(btn):
    return not btn.value

# ============= FILTRES =============
class Filter:
    """Combine filtre médian, détecteur stabilité et surveillance dérive"""
    def __init__(self, median_size=5, stability_window=20, stability_threshold=0.2):
        self.median_buf = []
        self.median_size = median_size
        self.stability_buf = []
        self.stability_window = stability_window
        self.stability_threshold = stability_threshold
        self.history = []
        self.mean = 0.0
        self.std = 1.0
        self.drift_estimate = 0.0
    
    def add(self, value):
        # Médian
        self.median_buf.append(value)
        if len(self.median_buf) > self.median_size:
            self.median_buf.pop(0)
        
        filtered = sorted(self.median_buf)[len(self.median_buf)//2] if self.median_buf else value
        
        # Rejet outliers
        if len(self.history) >= 20:
            z_score = abs(filtered - self.mean) / max(self.std, 0.5)
            if z_score > OUTLIER_REJECT_SIGMA:
                filtered = 0.7 * self.mean + 0.3 * filtered
        
        self.history.append(filtered)
        if len(self.history) > 100:
            self.history.pop(0)
        
        # Mise à jour stats
        if len(self.history) >= 5:
            self.mean = sum(self.history) / len(self.history)
            if len(self.history) >= 10:
                var = sum((x - self.mean)**2 for x in self.history) / len(self.history)
                self.std = math.sqrt(var) if var > 0 else 0.1
        
        # Stabilité
        self.stability_buf.append(filtered)
        if len(self.stability_buf) > self.stability_window:
            self.stability_buf.pop(0)
        
        # Dérive
        if len(self.history) >= 20:
            recent = self.history[-20:]
            old = self.history[:20] if len(self.history) >= 40 else self.history[:10]
            self.drift_estimate = sum(recent)/len(recent) - sum(old)/len(old)
        
        return filtered
    
    def is_stable(self):
        if len(self.stability_buf) < self.stability_window:
            return False
        mean = sum(self.stability_buf) / len(self.stability_buf)
        return max(abs(v - mean) for v in self.stability_buf) < self.stability_threshold
    
    def clear(self):
        self.median_buf.clear()
        self.stability_buf.clear()
        self.history.clear()
        self.mean = 0.0
        self.std = 1.0

# ============= MUX I2C =============
class MuxChannel:
    def __init__(self, i2c: busio.I2C, tca_addr: int = 0x70, channel: int = 0):
        self._i2c = i2c
        self._tca = tca_addr
        self._ch = min(7, max(0, channel))

    def _select(self):
        if self._i2c.try_lock():
            self._i2c.writeto(self._tca, bytes((1 << self._ch,)))
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

# ============= MINISCALE DRIVER =============
class MiniScale:
    REG_ADC = 0x00
    REG_WEIGHT = 0x10
    REG_BUTTON = 0x20
    REG_LED = 0x30
    REG_GAP = 0x40
    REG_RESET = 0x50
    REG_LPF = 0x80
    REG_AVG = 0x81
    REG_EMA = 0x82

    def __init__(self, i2c, address: int = 0x26, name: str = ""):
        self.i2c = i2c
        self.addr = address
        self._tare_offset = 0.0
        self.name = name
        self.led_order = "RGB"
        self.filter = Filter()

    def _write(self, reg: int, data: bytes):
        while not self.i2c.try_lock(): pass
        try:
            self.i2c.writeto(self.addr, bytes((reg,)) + data)
        finally:
            self.i2c.unlock()

    def _read(self, reg: int, n: int) -> bytes:
        while not self.i2c.try_lock(): pass
        try:
            self.i2c.writeto(self.addr, bytes((reg,)))
            buf = bytearray(n)
            self.i2c.readfrom_into(self.addr, buf)
            return bytes(buf)
        finally:
            self.i2c.unlock()

    def read_adc(self) -> int:
        return struct.unpack("<I", self._read(self.REG_ADC, 4))[0]

    def read_weight_raw(self) -> float:
        return struct.unpack("<f", self._read(self.REG_WEIGHT, 4))[0] - self._tare_offset

    def read_weight(self) -> float:
        return self.filter.add(self.read_weight_raw())

    def read_button(self) -> bool:
        return self._read(self.REG_BUTTON, 1)[0] == 0

    def set_led(self, r: int, g: int, b: int):
        payload = (g, r, b) if self.led_order == "GRB" else (r, g, b)
        self._write(self.REG_LED, bytes((p & 0xFF for p in payload)))

    def tare(self):
        samples = [self.read_weight_raw() for _ in (range(30), time.sleep(0.02))]
        self._tare_offset = sorted(samples)[len(samples)//2]
        self.filter.clear()

    def reset(self):
        self._write(self.REG_RESET, b"\x01")
        time.sleep(0.1)
        self.filter.clear()

    def is_stable(self):
        return self.filter.is_stable()

    def get_drift(self):
        return self.filter.drift_estimate

    def set_params(self, alpha=None, avg=None, lpf=None):
        if alpha is not None:
            self._write(self.REG_EMA, struct.pack("b", max(0, min(99, alpha))))
        if avg is not None:
            self._write(self.REG_AVG, struct.pack("b", max(0, min(50, avg))))
        if lpf is not None:
            self._write(self.REG_LPF, b"\x01" if lpf else b"\x00")

    def get_params(self):
        return (struct.unpack("b", self._read(self.REG_EMA, 1))[0],
                struct.unpack("b", self._read(self.REG_AVG, 1))[0],
                self._read(self.REG_LPF, 1) == b"\x01")

# ============= AFFICHAGE =============
display = board.DISPLAY
display.auto_refresh = True
main_group = displayio.Group()

def mk_label(text, x, y, scale=2, color=0xFFFFFF):
    return label.Label(terminalio.FONT, text=text, color=color, x=x, y=y, scale=scale)

labels = {
    'title': mk_label("Balances EVAPORATION x5", 6, 20, scale=2, color=0x00FFD0),
    'focus': mk_label("Ch:--  ----.-- g", 6, 58, scale=3),
    'adc': mk_label("ADC: -----", 6, 98, scale=2, color=0xA0A0A0),
    'filter': mk_label("", 6, 124, scale=1, color=0x80FF80),
    'button': mk_label("", 6, 140, scale=1, color=0xFFFF80),
    'status': mk_label("", 6, 158, scale=1, color=0xFFB070),
    'list': mk_label("", 6, 188, scale=1),
    'hint': mk_label("[L/R]=Switch [A]=Tare [B]=Calib [C]=Set [PRESS]=Log", 6, 226, scale=1, color=0x80B0FF)
}

for lbl in labels.values():
    main_group.append(lbl)
display.root_group = main_group

# ============= I2C SETUP =============
root_i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

def detect_tca():
    while not root_i2c.try_lock(): pass
    try:
        addrs = set(root_i2c.scan())
    finally:
        root_i2c.unlock()
    for a in range(0x70, 0x78):
        if a in addrs:
            return a
    raise RuntimeError("TCA9548A non trouvé")

MUX_ADDR = detect_tca()

def select_mux_channel(ch: int):
    while not root_i2c.try_lock(): pass
    try:
        root_i2c.writeto(MUX_ADDR, bytes((1 << min(7, ch),)))
    finally:
        root_i2c.unlock()
    time.sleep(0.08)

def probe_scale(addr=0x26):
    try:
        while not root_i2c.try_lock(): pass
        try:
            root_i2c.writeto(addr, bytes((0x10,)))
            buf = bytearray(4)
            root_i2c.readfrom_into(addr, buf)
            return True
        finally:
            root_i2c.unlock()
    except:
        return False

def detect_scales():
    found = []
    for ch in range(8):
        select_mux_channel(ch)
        if probe_scale():
            found.append(ch)
    return found

CHANNELS = detect_scales()[:5]
scales = []
active_idx = 0

def rebuild_scales():
    global scales, active_idx
    scales = []
    for ch in CHANNELS:
        mux = MuxChannel(root_i2c, tca_addr=MUX_ADDR, channel=ch)
        try:
            s = MiniScale(mux, name=f"S{ch}")
            s.set_params(alpha=DEFAULT_ALPHA, avg=DEFAULT_AVG, lpf=DEFAULT_LPF)
            scales.append({'ch': ch, 'scale': s, 'led': "RGB", 'cal': False})
        except:
            pass
    active_idx = 0 if scales else -1

rebuild_scales()
labels['status'].text = f"Balances: {[s['ch'] for s in scales]}"
time.sleep(1)

# ============= CALIBRATION =============
def avg_adc_samples(scale, samples=SAMPLES_PER_POINT, prefix=""):
    mean = M2 = 0.0
    for k in range(samples):
        x = float(scale.read_adc())
        delta = x - mean
        mean += delta / (k + 1)
        M2 += delta * (x - mean)
        if k % max(1, samples//10) == 0:
            sigma = math.sqrt(M2 / k) if k >= 1 else 0
            labels['status'].text = f"{prefix} {int(100*k/samples)}%  σ≈{sigma:.0f}"
        time.sleep(SAMPLE_DELAY)
    return int(round(mean))

def calibrate_2pt(scale, ch):
    labels['status'].text = f"[S{ch}] Reset..."
    scale.reset()
    time.sleep(0.8)
    
    labels['status'].text = f"[S{ch}] 0g"
    adc0 = avg_adc_samples(scale, prefix=f"[S{ch}] 0g")
    
    labels['status'].text = f"[S{ch}] Ajouter 100g puis B"
    while not (debouncers['B'].update(pressed(BTN_B))):
        time.sleep(0.05)
    
    labels['status'].text = f"[S{ch}] 100g"
    adc1 = avg_adc_samples(scale, prefix=f"[S{ch}] 100g")
    
    gap = (adc1 - adc0) / 100.0
    scale._write(scale.REG_GAP, struct.pack("<f", gap))
    labels['status'].text = f"[S{ch}] OK"
    
    for s in scales:
        if s['ch'] == ch:
            s['cal'] = True
    time.sleep(1)

# ============= MENU GÉNÉRIQUE =============
def menu(items, get_value=None, set_value=None, callback=None, title_prefix=""):
    sel = 0
    while True:
        val = get_value(sel) if get_value else None
        val_str = f" = {val}" if val is not None else ""
        labels['status'].text = f"{title_prefix} {items[sel]}{val_str}"
        
        if debouncers['UP'].update(pressed(JOY['UP'])):
            sel = (sel - 1) % len(items)
        if debouncers['DOWN'].update(pressed(JOY['DOWN'])):
            sel = (sel + 1) % len(items)
        
        if debouncers['LEFT'].update(pressed(JOY['LEFT'])) and set_value:
            set_value(sel, -1)
        if debouncers['RIGHT'].update(pressed(JOY['RIGHT'])) and set_value:
            set_value(sel, +1)
        
        if debouncers['B'].update(pressed(BTN_B)) and callback:
            callback(sel)
        if debouncers['C'].update(pressed(BTN_C)):
            break
        
        time.sleep(0.05)

# ============= BOUCLE PRINCIPALE =============
LOG_ENABLED = False
LOG_FILE = None
last_log = 0

while True:
    now = time.monotonic()
    
    # Navigation
    if debouncers['LEFT'].update(pressed(JOY['LEFT'])):
        active_idx = (active_idx - 1) % len(scales) if scales else -1
    if debouncers['RIGHT'].update(pressed(JOY['RIGHT'])):
        active_idx = (active_idx + 1) % len(scales) if scales else -1
    
    if active_idx >= 0 and scales:
        s_data = scales[active_idx]
        ch, scale = s_data['ch'], s_data['scale']
        
        try:
            w = scale.read_weight()
            a = scale.read_adc()
            btn = scale.read_button()
            stable = scale.is_stable()
            drift = scale.get_drift()
        except:
            w = a = drift = 0
            stable = btn = False
        
        # Affichage
        marker = " [OK]" if stable else ""
        labels['focus'].text = f"Ch:{ch}  {w:7.2f}g{marker}"
        labels['adc'].text = f"ADC:{a}"
        labels['button'].text = f"Btn[S{ch}]: {'PRESS' if btn else '-'}"
        labels['list'].text = " ".join(f"S{s['ch']}:{s['scale'].read_weight():.1f}" for s in scales)
        labels['filter'].text = f"Drift:{drift:.2f}g σ:{scale.filter.std:.2f}g Cal:{'OK' if s_data['cal'] else 'NO'}"
        labels['title'].text = f"EVAPORATION x{len(scales)} [S{ch}] Log:{'ON' if LOG_ENABLED else 'OFF'}"
        
        # Actions
        if debouncers['A'].update(pressed(BTN_A)):
            scale.tare()
            labels['status'].text = f"[S{ch}] Tare OK"
            time.sleep(0.5)
        
        if debouncers['B'].update(pressed(BTN_B)):
            calibrate_2pt(scale, ch)
        
        if debouncers['C'].update(pressed(BTN_C)):
            menu(["EMA", "AVG", "LPF", "Quitter"],
                 get_value=lambda i: [scale.get_params()[0], scale.get_params()[1], scale.get_params()[2], ""][i],
                 set_value=lambda i, d: scale.set_params(alpha=scale.get_params()[0]+d) if i==0 else None,
                 title_prefix=f"[S{ch}] Settings")
    
    # Logging
    if debouncers['PRESS'].update(pressed(JOY['PRESS'])):
        LOG_ENABLED = not LOG_ENABLED
        labels['status'].text = f"Log {'ON' if LOG_ENABLED else 'OFF'}"
        time.sleep(0.5)
    
    if active_idx < 0 or not scales:
        labels['focus'].text = "Ch:--  ----.-- g"
        labels['list'].text = "Aucune balance"
        time.sleep(0.5)
    
    time.sleep(0.03)
