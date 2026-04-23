from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, run_task, StopWatch
from pybricks.hubs import PrimeHub




hub = PrimeHub()




lmg = Motor(Port.F, positive_direction=Direction.COUNTERCLOCKWISE)
rmg = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
hrk = Motor(Port.D) #Die Klappe hinten
#vrk = Motor(Port.A) # Vorne Hoch runter
#zdm = Motor(Port.C)
col = ColorSensor(Port.E)
radius = 31.2
drb = DriveBase(lmg, rmg, 62.4, 200)
drb.use_gyro(True)

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
    while abs(zdm.angle()) < distance:
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
    
'''
def drb_m_rmk(distance, speed, rmk_angle, rmk_speed):
    drb.settings(speed, 900, 90, 500)
    drb.straight(distance, Stop.HOLD, False)
    
    rmk.reset_angle(0)
    
    while not drb.done():
        if abs(rmk.angle()) < rmk_angle:
            rmk.run(rmk_speed)
        
        if Button.RIGHT in hub.buttons.pressed():
            rmk.brake()
            raise StopRun("ENDE")
        if Button.CENTER in hub.buttons.pressed():
            rmk.brake()
            wait(1)
            raise StopRun("ENDE GELÄNDE!")
    
    rmk.brake()

    '''

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

def lf(dist, speed):
    # Kalibrierte Werte (unverändert)
    BLACK = 14
    WHITE = 92
    threshold = (BLACK + WHITE) / 2.0   # = 53.0

    DRIVE_SPEED = 180   # mm/s

    # ---------- OPTIMIERTE PID‑PARAMETER ----------
    Kp = 0.4          # Deutlich kleiner, um Überreaktion zu vermeiden
    Ki = 0.005          # Nur ganz leichtes Nachregeln
    Kd = 0.15        # Starke Dämpfung gegen Ruckeln
    
    # Begrenzung des Lenkeinschlags
    MAX_TURN_RATE = 90  # Grad/s – verhindert extremes Einlenken

    deviation = 0.0
    last_deviation = 0.0
    acc_deviation = 0.0
    dt = 0.03           # 30 ms pro Zyklus (≈ 33 Hz)

    print("Kp =", Kp, "Ki =", Ki, "Kd =", Kd)

    while True:
        # Abweichung berechnen (negativ = zu dunkel / auf Schwarz)
        deviation = col.reflection() - threshold

        # --- PID‑Berechnung ---
        diff = deviation - last_deviation
        acc_deviation += deviation * dt

        P_control = Kp * deviation
        I_control = Ki * acc_deviation
        D_control = Kd * diff / dt

        # Integral begrenzen (Anti‑Windup)
        I_control = max(-30, min(30, I_control))

        turn_rate = P_control + I_control + D_control

        # --- Lenkrate hart deckeln ---
        turn_rate = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, turn_rate))

        # --- Motoren ansteuern ---
        drb.drive(-DRIVE_SPEED, turn_rate)

        last_deviation = deviation
        wait(dt * 1000)   # dt in Millisekunden


def klappe():
    global isKlappeOben
    isKlappeOben = True
    if isKlappeOben == True:
        klapp(90, -300)
        isKlappeOben=False
    if isKlappeOben == False:
        klapp(90, 300)
        isKlappeOben=True

def sammeln():
    hrv(1,-300)
    gdd(60,300)
    hrv(10,-300)
    gdd(60,-300)
    hrv(60,300)

#Programmstart

hub.imu.reset_heading(0)
drb.reset()
klappe()

    




#wait(10000)

'''
m(100,500)
# vorne aufsammeln 
m(60,500)
# vorne aufsammeln 
m(96,500)
# vorne aufsammeln
m(60,500)
# vorne aufsammeln
t(90,400)
m(200,500)
t(-90,400)
m(90,500)
t(90,400)
lf(350,400)
m(350,500)
#vorne ablassen
m(-60,500)
#vorne ablassen
m(-60,500)
#vorne ablassen
m(-60,500)
#vorne ablassen
'''

#wait(500)

