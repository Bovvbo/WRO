from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait

hub = PrimeHub()
motor = Motor(Port.C)

motor.run_time(500, 2000, wait=True)
wait(100)  # Kurze Pause für den internen Zustandswechsel
motor.run_time(-500, 2000, wait=True)
wait(2000)  # Warten, bis der zweite Lauf fertig ist