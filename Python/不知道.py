from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, run_task
from pybricks.hubs import PrimeHub

hub = PrimeHub()

hub.system.set_stop_button({Button.BLUETOOTH})

class StopRun(Exception):
    def __init__(self, message: str = "", stop_program: bool = False):
        super().__init__(message)
        self.message = message
        self.stop_program = stop_program

lmg = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
rmg = Motor(Port.E, positive_direction=Direction.CLOCKWISE)
#lmk = Motor(Port.F)
#rmk = Motor(Port.B)
col = ColorSensor(Port.D)
radius = 31.2
drb = DriveBase(lmg, rmg, 62.4, 200)
drb.use_gyro(True)
'''
def lmkmove(distance, speed):
    lmk.reset_angle(0)
    while abs(lmk.angle()) < distance:
        if Button.BLUETOOTH in hub.buttons.pressed():
            lmk.brake()
            raise StopRun("ENDE")
        if Button.CENTER in hub.buttons.pressed():
            lmk.brake()
            wait(1)
            raise StopRun("ENDE GELÄNDE!")
        lmk.run(speed)
    lmk.brake()

def rmkmove(distance, speed):
    rmk.reset_angle(0)
    while abs(rmk.angle()) < distance:
        if Button.BLUETOOTH in hub.buttons.pressed():
            rmk.brake()
            raise StopRun("ENDE")
        if Button.CENTER in hub.buttons.pressed():
            rmk.brake()
            wait(1)
            raise StopRun("ENDE GELÄNDE!")
        rmk.run(speed)
    rmk.brake()
    '''

def drb_m(distance,speed,acceleration=900):
    drb.settings(speed,acceleration,90, 500)
    drb.straight(distance*-1,Stop.BRAKE,True)

def drb_t(angle,speed,acceleration=500):
    drb.settings(400,400,speed,acceleration)
    drb.turn(angle,Stop.BRAKE,True)

def drb_k(radius, angle, speed, acceleration=500):
    drb.use_gyro(False)
    drb.settings(straight_speed=speed, straight_acceleration=acceleration, turn_rate=100, turn_acceleration=acceleration)
    drb.curve(-radius, -angle, wait=False)
    while not drb.done():
        wait(10)
    drb.use_gyro(True)
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

def lf(dist, start_speed, end_speed):

    Kp = 2.4
    target = 52.5   # Linienwert anpassen

    speed = start_speed

    lmg.reset_angle(0)

    # -------- Ramp Up --------
    while speed < end_speed:

        speed += 20

        error = target - col.reflection()
        correction = Kp * error

        left = speed - correction
        right = speed + correction

        lmg.run(left)
        rmg.run(right)

        wait(30)

    # -------- Strecke fahren --------
    start_angle = lmg.angle()

    while abs(lmg.angle() - start_angle) < dist:

        error = target - col.reflection()
        correction = Kp * error

        left = speed - correction
        right = speed + correction

        lmg.run(left)
        rmg.run(right)

        wait(30)

    # -------- Ramp Down --------
    while speed > start_speed:

        speed -= 20

        error = target - col.reflection()
        correction = Kp * error

        left = speed - correction
        right = speed + correction

        lmg.run(left)
        rmg.run(right)

        wait(30)

    lmg.stop()
    rmg.stop()


#Programmstart

hub.imu.reset_heading(0)
drb.reset()
drb_m(500, 800)
drb_t(90, 400)
drb_k(-100, 90, 400)
lf(500, 200, 400)

