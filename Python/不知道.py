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
    BLACK = 14
    WHITE = 92
    threshold = (BLACK + WHITE) / 2.0

    Kp = 0.4
    Ki = 0.005
    Kd = 0.15
    MAX_TURN_RATE = 90

    dt = 0.03                     # 30 ms pro Zyklus
    drive_time = dist / speed     # benötigte Fahrzeit in Sekunden
    required_loops = int(drive_time / dt)   # Anzahl Schleifendurchläufe

    print(f"Fahre {dist} mm mit {speed} mm/s -> {drive_time:.2f} s = {required_loops} Loops")

    deviation = 0.0
    last_deviation = 0.0
    acc_deviation = 0.0

    loop_count = 0

    while loop_count < required_loops:
        deviation = col.reflection() - threshold

        diff = deviation - last_deviation
        acc_deviation += deviation * dt

        P_control = Kp * deviation
        I_control = Ki * acc_deviation
        D_control = Kd * diff / dt

        I_control = max(-30, min(30, I_control))
        turn_rate = P_control + I_control + D_control
        turn_rate = max(-MAX_TURN_RATE, min(MAX_TURN_RATE, turn_rate))

        drb.drive(-speed, turn_rate)   # Jetzt mit variablem speed!

        last_deviation = deviation
        wait(dt * 1000)

        loop_count += 1

    drb.stop()   # nach der gewünschten Anzahl Schleifen anhalten

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
m(220,500)
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

