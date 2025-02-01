from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *

class MyRobot2(RCJSoccerRobot):
    def ForwardAI(self):
        if self.yr > self.yb:
            move(self, self.xb+0.1, self.yb)
        else:
            move(self, self.xb, self.yb)
        # if self.yr > self.yb:
        #     if self.xr > self.xb:
        #         move(self, self.xb + 0.15, self.yb) 
        #     else:
        #         move(self, self.xb - 0.15, self.yb) 
        # elif self.arrived_to_target:
        #     # move(self, self.xb, self.yb)
        #     move(self, 0 , 0.75)
        #     if dist(self.xr, self.yr, self.xb, self.yb) > 0.2: 
        #         self.arrived_to_target = False
        # else:
        #     move(self, self.xt, self.yt)
        #     if dist(self.xr, self.yr, self.xt, self.yt) < 0.01: 
        #         self.arrived_to_target = True
    def run(self):
        define_variables(self)
        while self.robot.step(TIME_STEP) != -1:
            readData(self)
            xb = self.xb
            yb = self.yb
            ball_dist = dist(self.xr, self.yr, xb, yb)
            if ball_dist > 0.15 or not self.is_ball:
                moveAndLook(self, 0.55, -0.55, 0.2, 0)
            else:
                motor(self, 9, 9)