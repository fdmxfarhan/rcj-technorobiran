from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time

def dist(x1, y1, x2, y2):
    return math.sqrt((y1 - y2)**2 + (x1 - x2)**2)

class MyRobot1(RCJSoccerRobot):
    def readData(self):
        pos = self.get_gps_coordinates()
        self.heading = math.degrees(self.get_compass_heading())
        self.xr = pos[0]
        self.yr = pos[1]
        if self.robot.getName()[0] == 'B':
            self.xr *= -1
            self.yr *= -1
        if self.is_new_ball_data():
            self.is_ball = True
            ball_data = self.get_new_ball_data()
            self.ball_angle = math.degrees(math.atan2(ball_data['direction'][1], ball_data['direction'][0]))
            self.ball_distance = abs(0.01666666/(abs(ball_data['direction'][2])/math.sqrt(1 - ball_data['direction'][2]**2)))
            self.xb = -math.sin(math.radians(self.ball_angle + self.heading)) * self.ball_distance + self.xr
            self.yb =  math.cos(math.radians(self.ball_angle + self.heading)) * self.ball_distance + self.yr
        else:
            self.is_ball = False
    def move(self, x, y):
        angle = math.degrees(math.atan2(self.xr - x, y - self.yr))
        dest_angle = self.heading - angle
        if dest_angle > 5:
            self.right_motor.setVelocity(10)
            self.left_motor.setVelocity(-10)
        elif dest_angle < -5:
            self.right_motor.setVelocity(-10)
            self.left_motor.setVelocity(10)
        else:
            self.right_motor.setVelocity(10)
            self.left_motor.setVelocity(10)
    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    def run(self):
        self.xr = 0 
        self.yr = 0
        self.heading = 0
        self.xb = 0
        self.yb = 0
        self.ball_angle = 0
        self.ball_distance = 0
        self.is_ball = False
        self.yt = 0
        self.xt = 0
        self.arrived_to_target = False
        while self.robot.step(TIME_STEP) != -1:
            self.readData()

            if self.xb != 0 and self.yb != 0.75:
                m = (self.yb - 0.75) / self.xb
                b = 0.75
                self.yt = self.yb - 0.1
                self.xt = (self.yb - 0.1 - b)/m

            if self.is_ball:
                if self.arrived_to_target:
                    self.move(self.xb, self.yb)
                    if dist(self.xr, self.yr, self.xt, self.yt) > 0.2: 
                        self.arrived_to_target = False
                else:
                    self.move(self.xt, self.yt)
                    if dist(self.xr, self.yr, self.xt, self.yt) < 0.01: 
                        self.arrived_to_target = True
            else:
                self.move(0, 0)
           
            