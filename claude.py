from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, run_task, StopWatch
from pybricks.hubs import PrimeHub





hub = PrimeHub()




lmg = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
rmg = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
hrk = Motor(Port.D) #Die Klappe hinten
vrk = Motor(Port.A) # Vorne Hoch runter
zdm = Motor(Port.F)
col = ColorSensor(Port.E)
radius = 31.2
drb = DriveBase(lmg, rmg, 62.4, 200)
drb.use_gyro(True)


# ════════════════════════════════════════════════════════════════════════════
#  KONFIGURATION
# ════════════════════════════════════════════════════════════════════════════

class PIDConfig:
    """Alle Regelparameter zentral und dokumentiert."""
    KP            = 0.45     # Proportionalanteil
    KI            = 0.004    # Integralanteil
    KD            = 0.18     # Differentialanteil
    D_SMOOTH      = 0.65     # D-Term Low-Pass (0 = roh, 1 = eingefroren)
    I_CLAMP       = 25.0     # Anti-Windup: max. I-Anteil in °/s
    MAX_TURN      = 95       # Maximale Lenkrate in °/s
    DT            = 0.025    # Zielzykluszeit in Sekunden (25 ms)
    RAMP_LOOPS    = 15       # Loops für Geschwindigkeitsrampe beim Start


# ════════════════════════════════════════════════════════════════════════════
#  KALIBRIERUNG
# ════════════════════════════════════════════════════════════════════════════

def calibrate(samples: int = 20) -> tuple:
    """
    Misst Black- und White-Werte durch Überfahren der Linie.
    Roboter muss sich dabei langsam über die Linie bewegen.

    Returns:
        (black, white) als gemessene Extremwerte
    """
    readings = []
    for _ in range(samples):
        readings.append(col.reflection())
        wait(50)

    black = min(readings)
    white = max(readings)
    print(f"Kalibrierung: Schwarz={black}, Weiß={white}, Threshold={(black+white)/2:.1f}")
    return black, white


# ════════════════════════════════════════════════════════════════════════════
#  PID-REGLER (als eigene Klasse – wiederverwendbar)
# ════════════════════════════════════════════════════════════════════════════

class PIDController:
    """
    Generischer PID-Regler mit:
    - D-Term Low-Pass-Filterung
    - Anti-Windup per Clamping
    - Reset-Funktion für Mehrfachnutzung
    """

    def __init__(self, cfg: PIDConfig):
        self.cfg          = cfg
        self._integral    = 0.0
        self._prev_error  = 0.0
        self._smooth_deriv = 0.0

    def reset(self) -> None:
        """Regler zurücksetzen (z.B. vor jedem neuen Fahrauftrag)."""
        self._integral     = 0.0
        self._prev_error   = 0.0
        self._smooth_deriv = 0.0

    def update(self, error: float, dt: float) -> float:
        """
        Berechnet eine Steuergröße aus dem aktuellen Fehler.

        Args:
            error: Aktueller Regelabstand (Ist - Soll)
            dt:    Tatsächlich vergangene Zeit seit letztem Update in Sekunden

        Returns:
            Stellgröße (Lenkrate in °/s)
        """
        cfg = self.cfg

        # P-Anteil
        p = cfg.KP * error

        # I-Anteil mit Anti-Windup
        self._integral += error * dt
        i = cfg.KI * self._integral
        if i > cfg.I_CLAMP:
            i = cfg.I_CLAMP
            self._integral = cfg.I_CLAMP / cfg.KI   # Back-Calculation
        elif i < -cfg.I_CLAMP:
            i = -cfg.I_CLAMP
            self._integral = -cfg.I_CLAMP / cfg.KI

        # D-Anteil mit Low-Pass gegen Rauschen
        raw_deriv = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._smooth_deriv = (cfg.D_SMOOTH * self._smooth_deriv
                              + (1.0 - cfg.D_SMOOTH) * raw_deriv)
        d = cfg.KD * self._smooth_deriv

        self._prev_error = error

        # Stellgröße begrenzen
        output = p + i + d
        return max(-cfg.MAX_TURN, min(cfg.MAX_TURN, output))


# ════════════════════════════════════════════════════════════════════════════
#  HAUPTFUNKTION
# ════════════════════════════════════════════════════════════════════════════

def lf(
    dist:  float,
    speed: float,
    black: int = 14,
    white: int = 92,
    cfg:   PIDConfig = None
) -> bool:
    """
    Fährt 'dist' mm rückwärts entlang einer Linie.

    Args:
        dist:  Fahrstrecke in mm
        speed: Zielgeschwindigkeit in mm/s
        black: Sensorwert auf schwarzer Linie (aus Kalibrierung)
        white: Sensorwert auf weißem Untergrund (aus Kalibrierung)
        cfg:   PIDConfig-Objekt (Standard wird verwendet wenn None)

    Returns:
        True bei Erfolg, False bei ungültigen Parametern
    """

    # ── Validierung ──────────────────────────────────────────────────────────
    if dist <= 0 or speed <= 0:
        print(f"Fehler: dist={dist}, speed={speed} – beide müssen > 0 sein.")
        return False
    if black >= white:
        print(f"Fehler: black={black} muss kleiner als white={white} sein.")
        return False

    if cfg is None:
        cfg = PIDConfig()

    threshold      = (black + white) / 2.0
    drive_time     = dist / speed
    required_loops = int(drive_time / cfg.DT)

    print(f"Start: {dist} mm | {speed} mm/s | {drive_time:.2f} s | {required_loops} Loops")
    print(f"Threshold: {threshold:.1f} | KP={cfg.KP} KI={cfg.KI} KD={cfg.KD}")

    # ── Regler initialisieren ────────────────────────────────────────────────
    pid    = PIDController(cfg)
    timer  = StopWatch()
    prev_t = timer.time()

    # ── Regelschleife ────────────────────────────────────────────────────────
    for loop_count in range(required_loops):

        # Echtes dt messen (robuster als festes DT)
        now    = timer.time()
        real_dt = (now - prev_t) / 1000.0   # ms → s
        if real_dt <= 0:
            real_dt = cfg.DT                 # Fallback beim ersten Loop
        prev_t = now

        # Abweichung + Stellgröße
        error     = col.reflection() - threshold
        turn_rate = pid.update(error, real_dt)

        # Geschwindigkeitsrampe beim Anfahren (sanfterer Start)
        if loop_count < cfg.RAMP_LOOPS:
            ramp_factor = (loop_count + 1) / cfg.RAMP_LOOPS
            current_speed = speed * ramp_factor
        else:
            current_speed = speed

        drb.drive(-current_speed, turn_rate)

        # Adaptives Timing
        elapsed   = timer.time() - prev_t
        remaining = int(cfg.DT * 1000) - elapsed
        if remaining > 1:
            wait(remaining)

    # ── Stoppen ──────────────────────────────────────────────────────────────
    drb.stop()
    print(f"Ziel erreicht nach {timer.time() / 1000:.2f} s.")
    return True


# ════════════════════════════════════════════════════════════════════════════
#  BEISPIEL-AUFRUF
# ════════════════════════════════════════════════════════════════════════════

# Standard-Aufruf:
lf(500, 300)

# Mit eigener Kalibrierung:
# b, w = calibrate()
# lf(500, 150, black=b, white=w)

# Mit angepassten PID-Werten:
# my_cfg = PIDConfig()
# my_cfg.KP = 0.5
# lf(500, 150, cfg=my_cfg)