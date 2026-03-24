from pybricks.tools import wait, StopWatch, hub_menu
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import multitask, run_task
from pybricks.hubs import PrimeHub


hub = PrimeHub()

hub.system.set_stop_button({Button.BLUETOOTH})


session_watch = None  # StopWatch für die gesamte Session
session_start_time = None  # Startzeit in Millisekunden

class StopRun(Exception):
    def __init__(self, message: str = "", stop_program: bool = False):
        super().__init__(message)
        self.message = message
        self.stop_program = stop_program

lmg = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
rmg = Motor(Port.E, positive_direction=Direction.CLOCKWISE)
#lmk = Motor(Port.F)
#rmk = Motor(Port.B)
radius = 31.2
drb = DriveBase(lmg, rmg, 62.4, 200)
drb.use_gyro(True)


#Firebase startinfos
print("restart")
print("battery",hub.battery.voltage() / 1000)
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
    drb.straight(distance,Stop.HOLD,True)
    while not drb.done():
        if Button.RIGHT in hub.buttons.pressed():
            raise StopRun("ENDE")
        if Button.CENTER in hub.buttons.pressed():
            wait(1)
            raise StopRun("ENDE GELÄNDE!")

def drb_t(angle,speed,acceleration=500):
    drb.settings(400,400,speed,acceleration)
    drb.turn(angle,Stop.HOLD,True)
    

def drb_k(radius, angle, speed, acceleration=500):

    drb.settings(straight_speed=speed, straight_acceleration=acceleration, turn_rate=100, turn_acceleration=acceleration)
    drb.curve(radius, angle, wait=False)
    
    while not drb.done():
        if Button.BLUETOOTH in hub.buttons.pressed():
            raise StopRun("ENDE")
        if Button.CENTER in hub.buttons.pressed():
            wait(1)
            raise StopRun("ENDE GELÄNDE!")
        
        wait(10)

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


#test
drb_t(90, 300)