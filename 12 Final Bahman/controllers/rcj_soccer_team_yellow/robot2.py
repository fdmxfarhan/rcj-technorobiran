from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *

class MyRobot2(RCJSoccerRobot):
    def ForwardAI_movingball(self):
        if self.static_xb != 0 and self.static_yb != OPONENT_GOAL_Y:
            m = (self.static_yb - OPONENT_GOAL_Y) / self.static_xb
            b = OPONENT_GOAL_Y
            self.yt = self.static_yb - 0.1
            self.xt = (self.static_yb - 0.1 - b)/m
        self.ForwardAI()
    def ForwardAI(self):
        if self.yr > self.yb:
            if self.xr > self.xb:
                move(self, self.xb + 0.15, self.yb) 
            else:
                move(self, self.xb - 0.15, self.yb) 
        elif self.arrived_to_target:
            move(self, self.xb, self.yb)
            if dist(self.xr, self.yr, self.xb, self.yb) > 0.2: 
                self.arrived_to_target = False
        else:
            move(self, self.xt, self.yt)
            if dist(self.xr, self.yr, self.xt, self.yt) < 0.01: 
                self.arrived_to_target = True
    def run(self):
        define_variables(self)
        while self.robot.step(TIME_STEP) != -1:
            readData(self)
            if self.manual_control:
                manualControl(self)
            else:
                if self.is_ball:
                    if self.ball_stop_time > 3:
                        LackOffProgress(self)
                    else:
                        if self.xb >= 0 and self.yb > -0.5:
                            if self.xb < 0.3 and self.xb > -0.3:
                                move(self, 0, self.yb)
                            else:
                                move(self, 0, self.yb - 0.2)
                        else:
                            if self.ball_stop_time == 0:
                                self.ForwardAI()
                            else:
                                self.ForwardAI_movingball()
                else:
                    Formation(self)
           
            