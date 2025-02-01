from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time
from utils import *

class MyRobot1(RCJSoccerRobot):
    def GoalKeeperAI(self):
        if self.yb < -0.5 and (self.xb > 0.3 or self.yb < -0.3):
            if self.xb > 0:
                move(self, 0.4, self.yb)
            else:
                move(self, -0.4, self.yb)
        elif self.yr > -0.5 or self.yr < -0.55:
            move(self, clamp(self.xb, -0.4, 0.4), -0.55)
        else:
            if self.heading > 95: 
                motor(self, -10, 10)
            elif self.heading < 85:
                motor(self, 10, -10)
            else:
                if clamp(self.xb, -0.4, 0.4) > self.xr:
                    motor(self, -10, -10)
                else:
                    motor(self, 10, 10)
    def run(self):
        define_variables(self)
        game_started = False
        arrived_to_ball = False
        cnt = 0
        while self.robot.step(TIME_STEP) != -1:
            readData(self)
            xb = self.xb
            yb = self.yb
            ball_dist = dist(self.xr, self.yr, xb, yb)
            # print(self.sonar['front']/1000)
            if game_started:
                if ball_dist > 0.1 or not self.is_ball:
                    moveAndLook(self, 0.6, -0.6, 0, 0)
                else:
                    moveAndLook(self, 0.5, -0.5, 0, 0)
            else:
                if not arrived_to_ball:
                    xg = 0.45
                    yg = 0.5
                    yt = yb - 0.1
                    xt = xb
                    moveAndLook(self, xt, yt, xg, yg)
                    if dist(self.xr, self.yr, xt, yt) < 0.01:
                        cnt += 1
                        if cnt > 100: arrived_to_ball = True
                else:
                    motor(self, 10, 10)
                    if self.yr > -0.3:
                        game_started = True