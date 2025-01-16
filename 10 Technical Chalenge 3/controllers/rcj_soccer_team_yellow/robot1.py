from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *
## Technical Challenge 2
## برنامه ای بنیوسید که ربات در یک ماز حرکت کند و توپ را گل کند
class MyRobot1(RCJSoccerRobot):
    def run(self):
        define_variables(self)
        while self.robot.step(TIME_STEP) != -1:
            readData(self)
            yb = self.yb
            xb = self.xb

            if self.yb < -0.25:  # step 1
                xg = 0.5
                yg = -0.25
            elif self.yb < 0.25: # step 2
                xg = -0.5
                yg =  0.25
            else:                # step 3
                xg = 0
                yg = 2

            m = (yg - yb) / (xg - xb)
            b = yb - m * xb
            yt = yb - 0.1
            xt = (yt - b) / m
            
            if self.is_ball:
                if dist(xb, yb, -0.6, -0.2) < 0.06:
                    motor(self, -10, 10)
                elif dist(xb, yb, 0.15, 0.35) < 0.06:
                    motor(self, 10, -10)
                elif self.arrived_to_target: 
                    move(self, xb, yb)
                    if self.ball_distance > 0.15: 
                        self.arrived_to_target = False
                else: 
                    move(self, xt, yt)
                    if dist(self.xr, self.yr, xt, yt) < 0.1: 
                        self.arrived_to_target = True
            else:
                move(self, xb, yb)
                # stop(self)