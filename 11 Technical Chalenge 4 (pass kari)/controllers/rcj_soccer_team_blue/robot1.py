from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *

class MyRobot1(RCJSoccerRobot):
    def run(self):
        define_variables(self)
        while self.robot.step(TIME_STEP) != -1:
            readData(self)
            xb = self.xb
            yb = self.yb
            ball_dist = dist(self.xr, self.yr, xb, yb)
            if ball_dist > 0.15 or not self.is_ball:
                moveAndLook(self, -0.55, -0.55, 0, -0.2)
            else:
                motor(self, 9, 9)