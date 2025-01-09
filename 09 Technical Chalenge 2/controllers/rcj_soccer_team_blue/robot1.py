from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *

class MyRobot1(RCJSoccerRobot):
    def run(self):
        define_variables(self)
        while self.robot.step(TIME_STEP) != -1:
            readData(self)
            
            