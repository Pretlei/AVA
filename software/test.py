from serial_comms_comp_1 import RobotArm
from ik_solver_2 import IKController
import serial
import time
 
PORT = "COM6"

# x max 185/340, y max 340/-340
with RobotArm(PORT) as arm:
    controller = IKController(arm)
    arm.rest()
    controller.move_to(320, 100)
    time.sleep(3)
    arm.rest()
    