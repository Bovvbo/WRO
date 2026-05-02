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

BLACK          = 14
WHITE          = 92
THRESHOLD      = (BLACK + WHITE) / 2.0
 
KP             = 0.4
KI             = 0.005
KD             = 0.15
MAX_TURN_RATE  = 90
DT             = 0.03        # Zielzykluszeit in Sekunden
D_SMOOTH       = 0.7         # Low-Pass-Koeffizient für D-Term (0 = kein Filter)
I_CLAMP        = 30          # Maximaler I-Anteil

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
    timer= StopWatch()
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
def m(distance,speed,acceleration=600):
    drb.settings(speed,acceleration,90, 500)
    drb.straight(distance*-1,Stop.COAST,True)

def t(angle,speed,acceleration=500):
    drb.settings(400,400,speed,acceleration)
    drb.turn(angle,Stop.COAST,True)

def k(radius, angle, speed, acceleration=500):
    drb.settings(straight_speed=speed, straight_acceleration=acceleration, turn_rate=100, turn_acceleration=acceleration)
    drb.curve(radius, angle, wait=True)
    while not drb.done():
        wait(10)

def lf_s(dist, start_speed, end_speed):

    Kp = 2.4
    target = 52.5

    speed = start_speed

    lmg.reset_angle(0)

    # -------- Ramp Up --------
    while speed < end_speed:

        speed += 20

        error = target - col.reflection()
        correction = Kp * error

        left = -(speed + correction)
        right = -(speed - correction)

        lmg.run(left)
        rmg.run(right)

        wait(30)

    # -------- Strecke fahren --------
    start_angle = lmg.angle()

    while abs(lmg.angle() - start_angle) < dist:

        error = target - col.reflection()
        correction = Kp * error

        left = -(speed + correction)
        right = -(speed - correction)

        lmg.run(left)
        rmg.run(right)

        wait(30)

    # -------- Ramp Down --------
    while speed > start_speed:

        speed -= 20

        error = target - col.reflection()
        correction = Kp * error

        left = -(speed + correction)
        right = -(speed - correction)

        lmg.run(left)
        rmg.run(right)

        wait(30)

    lmg.stop()
    rmg.stop()

def lf(dist: float, speed: float) -> None:
    """
    Fährt 'dist' mm rückwärts entlang einer Linie mit 'speed' mm/s.
 
    Args:
        dist:  Fahrstrecke in mm (muss > 0 sein)
        speed: Fahrgeschwindigkeit in mm/s (muss > 0 sein)
    """
 
    # ── Eingabevalidierung ───────────────────────────────────────────────────
    if dist <= 0 or speed <= 0:
        print("Fehler: dist und speed müssen größer als 0 sein.")
        return
 
    # ── Laufvariablen ────────────────────────────────────────────────────────
    drive_time     = dist / speed
    required_loops = int(drive_time / DT)
    print(f"Fahre {dist} mm mit {speed} mm/s → {drive_time:.2f} s = {required_loops} Loops")
 
    deviation      = 0.0
    last_deviation = 0.0
    acc_deviation  = 0.0
    smooth_diff    = 0.0      # geglättete Ableitung für D-Term
    loop_count     = 0
 
    timer = StopWatch()       # für adaptives Timing
 
    # ── Regelschleife ────────────────────────────────────────────────────────
    while loop_count < required_loops:
        timer.reset()
 
        # Abweichung vom Schwellwert
        deviation      = col.reflection() - THRESHOLD
        diff           = deviation - last_deviation
 
        # D-Term: Low-Pass-Filter gegen Sensorrauschen
        smooth_diff    = D_SMOOTH * smooth_diff + (1.0 - D_SMOOTH) * diff
 
        # Integrator aufaddieren
        acc_deviation += deviation * DT
 
        # PID-Anteile berechnen
        P_control = KP * deviation
        I_control = KI * acc_deviation
        D_control = KD * smooth_diff / DT
 
        # Anti-Windup: I-Anteil begrenzen
        I_control = max(-I_CLAMP, min(I_CLAMP, I_control))
 
        # Stellgröße zusammensetzen und begrenzen
        turn_rate = P_control + I_control + D_control
        turn_rate = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, turn_rate))
 
        drb.drive(-speed, turn_rate)
 
        last_deviation = deviation
        loop_count    += 1
 
        # Adaptives Timing: verbleibende Zeit im Zyklus abwarten
        elapsed_ms = timer.time()
        remaining  = int(DT * 1000) - elapsed_ms
        if remaining > 0:
            wait(remaining)
 
    # ── Sauber stoppen ───────────────────────────────────────────────────────
    drb.stop()
    print("Strecke abgefahren.")

def klappe():
    global isKlappeOben
    if isKlappeOben == True:
        klapp(90, -300)
        isKlappeOben=False
    else:
        klapp(90, 300)
        isKlappeOben=True

def sammeln():
    hrv(250,-700)
    gdd(100,300)
    hrv(250,-700)
    hrv(150,700)
    hrv(150,-700)
    gdd(100,-300)
    hrv(500,700)

def abladen():
    hrv(350,-700)
    gdd(120,300)
    wakeln()
    hrv(150,700)
    gdd(120,-300)
    hrv(200,700)

def wakeln():
    m(20,300)
    m(-20,300)
    

#Programmstart

hub.imu.reset_heading(0)
drb.reset()
isKlappeOben = True



m(127,500)


sammeln()
m(60,500)
sammeln()
m(95,500)
sammeln()
m(60,500)
sammeln()

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



