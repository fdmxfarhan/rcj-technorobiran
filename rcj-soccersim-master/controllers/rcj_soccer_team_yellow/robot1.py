from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time

class MyRobot1(RCJSoccerRobot):
    def readData(self):
        pos = self.get_gps_coordinates()
        self.heading = math.degrees(self.get_compass_heading())
        self.xr = pos[0]
        self.yr = pos[1]
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
        while self.robot.step(TIME_STEP) != -1:
            self.readData()
            self.move(0, 0)
           
            