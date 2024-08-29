from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time

OPONENT_GOAL_Y = 1.5

def dist(x1, y1, x2, y2):
    return math.sqrt((y1 - y2)**2 + (x1 - x2)**2)

class MyRobot2(RCJSoccerRobot):
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
        ###################################### Find Behind Ball Spot
        if self.xb != 0 and self.yb != OPONENT_GOAL_Y:
                m = (self.yb - OPONENT_GOAL_Y) / self.xb
                b = OPONENT_GOAL_Y
                self.yt = self.yb - 0.1
                self.xt = (self.yb - 0.1 - b)/m
        ###################################### Ersal Data be team
        self.send_data_to_team({
            'is_ball': self.is_ball,
            'xb': self.xb,
            'yb': self.yb,
            'xr': self.xr,
            'yr': self.yr,
            'id': self.robot.getName()[1]
        })
        ###################################### Daryaft Data az team
        while self.is_new_team_data():
            team_data = self.get_new_team_data()['robot_id']
            if not self.is_ball and team_data['is_ball']:
                self.xb = team_data['xb']
                self.yb = team_data['yb']
                self.is_ball = True

    def motor(self, vl, vr):
        if vr > 10: vr = 10
        if vr <-10: vr =-10
        if vl > 10: vl = 10
        if vl <-10: vl =-10
        self.right_motor.setVelocity(vr)
        self.left_motor.setVelocity(vl)
    def move(self, x, y):
        angle = math.degrees(math.atan2(self.xr - x, y - self.yr))
        dest_angle = self.heading - angle
        if dest_angle > 180: dest_angle -= 360
        if dest_angle <-180: dest_angle += 360

        if dist(self.xr, self.yr, x, y) < 0.02 and not self.is_ball:
            self.stop()
        elif dest_angle > -90 and dest_angle < 90:
            self.motor(10 - dest_angle*0.4, 10 + dest_angle*0.4)
        else:
            if dest_angle >= 0: dest_angle -= 180
            else:               dest_angle += 180
            self.motor(-10 - dest_angle*0.4, -10 + dest_angle*0.4)
    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    def ForwardAI(self):
        if self.yr > self.yb:
            if self.xr > self.xb:
                self.move(self.xb + 0.15, self.yb) 
            else:
                self.move(self.xb - 0.15, self.yb) 
        elif self.arrived_to_target:
            # self.move(self.xb, self.yb)
            self.move(0 , 0.75)
            if dist(self.xr, self.yr, self.xb, self.yb) > 0.2: 
                self.arrived_to_target = False
        else:
            self.move(self.xt, self.yt)
            if dist(self.xr, self.yr, self.xt, self.yt) < 0.01: 
                self.arrived_to_target = True
    def Formation(self):
        if self.robot.getName()[1] == '1':
            self.move(0, -0.6)
        if self.robot.getName()[1] == '2':
            self.move(-0.3, -0.2)
        if self.robot.getName()[1] == '3':
            self.move(0.3, -0.2)
    def GoalKeeperAI(self):
        self.move(self.xb, -0.6)
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
            


            if self.is_ball:
                self.ForwardAI()
                # self.GoalKeeperAI()
            else:
                self.Formation()
           
            