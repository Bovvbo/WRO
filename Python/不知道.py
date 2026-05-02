from pybricks.tools import wait, StopWatch
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Direction, Port, Stop
from pybricks.robotics import DriveBase
from pybricks.hubs import PrimeHub

hub = PrimeHub()

lmg = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
rmg = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
hrk = Motor(Port.F)
vrk = Motor(Port.A)
zdm = Motor(Port.C)
col = ColorSensor(Port.E)
drb = DriveBase(lmg, rmg, 62.4, 200)
drb.use_gyro(True)


# ═══════════════════════════════════════════════════════════════════════
#  HILFSFUNKTIONEN
# ═══════════════════════════════════════════════════════════════════════

def klapp(distance, speed):
    hrk.reset_angle(0)
    timer = StopWatch()
    last_angle = hrk.angle()
    while abs(hrk.angle()) < distance:
        current_angle = hrk.angle()
        if abs(current_angle - last_angle) > 0.5:
            last_angle = current_angle
            timer.reset()
        if timer.time() > 500:
            hrk.brake()
            return
        hrk.run(speed)
    hrk.brake()

def hrv(distance, speed):
    vrk.reset_angle(0)
    while abs(vrk.angle()) < distance:
        vrk.run(speed)
    vrk.brake()

def gdd(distance, speed):
    zdm.reset_angle(0)
    timer = StopWatch()
    last_angle = zdm.angle()
    while abs(zdm.angle()) < distance:
        current_angle = zdm.angle()
        if abs(current_angle - last_angle) > 0.5:
            last_angle = current_angle
            timer.reset()
        if timer.time() > 500:
            zdm.brake()
            return
        zdm.run(speed)
    zdm.brake()

def m(distance, speed, acceleration=600):
    drb.settings(speed, acceleration, 90, 500)
    drb.straight(distance * -1, Stop.COAST, True)

def t(angle, speed, acceleration=500):
    drb.settings(speed, acceleration, speed, acceleration)
    drb.turn(angle, Stop.COAST, True)

def k(radius, angle, speed, acceleration=500):
    drb.settings(straight_speed=speed, straight_acceleration=acceleration, turn_rate=100, turn_acceleration=acceleration)
    drb.curve(radius, angle, wait=True)

def klappe():
    global isKlappeOben
    if isKlappeOben==True:
        klapp(120, 500)
        isKlappeOben = False
    else:
        klapp(120, -500)
        isKlappeOben = True

def sammeln():
    hrv(250, 500)
    #wait(500)
    #hrv(100, 300)
    gdd(150, 300)
    hrv(200, 500)
    #hrv(200, -700)
    #hrv(200, 700)
    gdd(200, -300)
    hrv(450, -700)

def abladen():
    hrv(350, 700)
    gdd(120, 300)
    wakeln()
    hrv(150, -700)
    gdd(120, -300)
    hrv(200, -700)

def wakeln():
    m(20, 300)
    m(-20, 300)


# ═══════════════════════════════════════════════════════════════════════
#  LINIENFOLGER
# ═══════════════════════════════════════════════════════════════════════

class PIDConfig:
    """Alle Regelparameter zentral und dokumentiert."""
    KP           = 0.35
    KI           = 0.003
    KD           = 0.18
    D_SMOOTH     = 0.65
    I_CLAMP      = 25.0
    MAX_TURN     = 120
    DT           = 0.010
    RAMP_LOOPS   = 15
    LINE_LOSS_MS = 1000
    ALIGN_SPEED  = 80
    ALIGN_LOOPS  = 30
    ALIGN_THRESH = 3.0


class PIDController:
    def __init__(self, cfg: PIDConfig):
        self.cfg           = cfg
        self._integral     = 0.0
        self._prev_error   = 0.0
        self._smooth_deriv = 0.0

    def reset(self) -> None:
        self._integral     = 0.0
        self._prev_error   = 0.0
        self._smooth_deriv = 0.0

    def update(self, error: float, dt: float) -> float:
        cfg = self.cfg
        p = cfg.KP * error

        self._integral += error * dt
        i = cfg.KI * self._integral
        if i > cfg.I_CLAMP:
            i = cfg.I_CLAMP
            self._integral = cfg.I_CLAMP / cfg.KI
        elif i < -cfg.I_CLAMP:
            i = -cfg.I_CLAMP
            self._integral = -cfg.I_CLAMP / cfg.KI

        raw_deriv = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._smooth_deriv = (cfg.D_SMOOTH * self._smooth_deriv
                              + (1.0 - cfg.D_SMOOTH) * raw_deriv)
        d = cfg.KD * self._smooth_deriv
        self._prev_error = error

        output = p + i + d
        return max(-cfg.MAX_TURN, min(cfg.MAX_TURN, output))


def lf(
    dist:  float,
    speed: float,
    black: int = 14,
    white: int = 92,
    side:  str = "left",
    cfg:   PIDConfig = None
) -> bool:
    """
    Fährt 'dist' mm rückwärts entlang einer Linie.
    Gyro wird während lf automatisch deaktiviert und danach wieder aktiviert.
    """

    if dist <= 0 or speed <= 0:
        print(f"Fehler: dist={dist}, speed={speed} – beide müssen > 0 sein.")
        return False
    if black >= white:
        print(f"Fehler: black={black} muss kleiner als white={white} sein.")
        return False
    if side not in ("left", "right"):
        print(f"Fehler: side='{side}' – muss 'left' oder 'right' sein.")
        return False

    if cfg is None:
        cfg = PIDConfig()

    # Gyro aus für lf
    drb.use_gyro(False)

    threshold  = (black + white) / 2.0
    side_sign  = 1 if side == "left" else -1
    max_ramp   = max(1, int(dist / speed / cfg.DT) - 1)
    ramp_loops = min(cfg.RAMP_LOOPS, max_ramp)

    print(f"Start: {dist} mm | {speed} mm/s | Threshold: {threshold:.1f} | Seite: {side}")

    pid = PIDController(cfg)

    # ── Ausricht-Phase ───────────────────────────────────────────────
    print("Ausrichten...")
    align_timer = StopWatch()
    prev_t = align_timer.time()

    for _ in range(cfg.ALIGN_LOOPS):
        now     = align_timer.time()
        real_dt = max((now - prev_t) / 1000.0, cfg.DT)
        prev_t  = now

        reflection = col.reflection()
        error      = (reflection - threshold) * side_sign
        turn_rate  = pid.update(error, real_dt)

        drb.drive(-cfg.ALIGN_SPEED, turn_rate)

        if abs(error) < cfg.ALIGN_THRESH:
            break

        elapsed   = align_timer.time() - now
        remaining = int(cfg.DT * 1000) - elapsed
        if remaining > 1:
            wait(remaining)

    print("Ausgerichtet – Hauptfahrt startet.")

    # ── Reset für Hauptfahrt ─────────────────────────────────────────
    pid.reset()
    drb.reset()
    timer  = StopWatch()
    prev_t = timer.time()
    line_loss_timer  = StopWatch()
    line_loss_active = False

    # ── Hauptfahrt ───────────────────────────────────────────────────
    while abs(drb.distance()) < dist:

        now     = timer.time()
        real_dt = (now - prev_t) / 1000.0
        if real_dt <= 0:
            real_dt = cfg.DT
        prev_t = now

        reflection = col.reflection()
        error      = (reflection - threshold) * side_sign
        turn_rate  = pid.update(error, real_dt)

        if reflection < black - 5 or reflection > white + 5:
            if not line_loss_active:
                line_loss_active = True
                line_loss_timer.reset()
            elif line_loss_timer.time() > cfg.LINE_LOSS_MS:
                print("Linienverlust! Abbruch.")
                drb.stop()
                drb.use_gyro(True)
                return False
        else:
            line_loss_active = False

        loop_count    = int(abs(drb.distance()) / speed / cfg.DT)
        ramp_factor   = min(1.0, (loop_count + 1) / ramp_loops)
        current_speed = speed * ramp_factor

        drb.drive(-current_speed, turn_rate)

        elapsed   = timer.time() - now
        remaining = int(cfg.DT * 1000) - elapsed
        if remaining > 1:
            wait(remaining)

    drb.stop()
    # Gyro wieder an für alle anderen Funktionen
    drb.use_gyro(True)
    print(f"Ziel erreicht nach {timer.time() / 1000:.2f} s.")
    return True


# ═══════════════════════════════════════════════════════════════════════
#  PROGRAMMSTART
# ═══════════════════════════════════════════════════════════════════════

hub.imu.reset_heading(0)
drb.reset()
isKlappeOben = True

#lf(400, 300, side="left")

m(135,500)

gdd(100,300)

hrv(100,200)


sammeln()
m(62,500)
sammeln()
m(100,500)
sammeln()
m(62,500)
sammeln()

hrv(150,-700)

m(-400,500)
t(-90,400)
m(200,500)
m(-720,500)

klappe()

m(300,250)
klappe()

m(100,500)  # 1
t(-45,400)
m(-130,500)
t(45,400)

m(-400,500)

t(45,400)
m(-120,500)
t(-45,400)
m(-150,500)

klappe()

t(-90,400) # 2
m(-100,500)
t(90,400)
m(-600,500)
t(-100,400)
m(-50,500)

klappe()

m(50,500)

t(100,400)



'''
m(-60,500)
t(90,400)
m(300,500)
t(-90,400)
m(225,500)
t(90,400)

lf(400,180)

#k(300,3,500)
#k(-300,3,500)
t(7,400)
m(10,500)
t(-7,400)
m(300,500)
#m(680,500)
abladen()
m(-50,500)
abladen()
m(-50,500)
abladen()
m(-50,500)
abladen()

'''

