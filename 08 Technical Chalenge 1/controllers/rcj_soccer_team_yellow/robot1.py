from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *

class MyRobot1(RCJSoccerRobot):
    def run(self):
        define_variables(self)
        target_is_blue = True
        stop(self)
        timeout = time.time()
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                readData(self)
                if self.is_ball and time.time() - timeout > 2:
                    if self.yb > 0.75 and self.yb < 0.8:  target_is_blue = False
                    if self.yb < -0.75 and self.yb < 0.8: target_is_blue = True
                    if target_is_blue:
                        if self.arrived_to_target:
                            move(self, self.xb, self.yb)
                            if dist(self.xr, self.yr, self.xb, self.yb) > 0.2: 
                                self.arrived_to_target = False
                        else:
                            move(self, self.xt, self.yt)
                            if dist(self.xr, self.yr, self.xt, self.yt) < 0.03: 
                                self.arrived_to_target = True
                    else:
                        if self.yr < self.yb:
                            move(self, self.xb-0.4, self.yb+0.2)
                        elif self.arrived_to_target:
                            move(self, self.xb, self.yb)
                            if dist(self.xr, self.yr, self.xb, self.yb) > 0.2: 
                                self.arrived_to_target = False
                        else:
                            move(self, self.xt2, self.yt2)
                            if dist(self.xr, self.yr, self.xt2, self.yt2) < 0.03: 
                                self.arrived_to_target = True
                else:
                    stop(self)