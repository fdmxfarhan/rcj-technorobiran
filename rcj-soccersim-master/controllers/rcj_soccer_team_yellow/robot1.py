from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time

class MyRobot1(RCJSoccerRobot):
    def readData(self):
        pos = self.get_gps_coordinates()
        self.heading = math.degrees(self.get_compass_heading())
        self.xr = pos[0]
        self.yr = pos[1]

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
    def run(self):
        self.xr = 0 
        self.yr = 0
        self.heading = 0
        self.xb = 0
        self.yb = 0
        self.ball_angle = 0
        self.ball_distance = 0
        self.is_ball = False
        while self.robot.step(TIME_STEP) != -1:
            self.readData()
            if self.is_ball:
                self.move(self.xb, self.yb)
            else:
                self.move(0, 0)
           
            