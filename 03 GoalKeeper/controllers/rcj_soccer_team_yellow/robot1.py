from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time

OPONENT_GOAL_Y = 1.5

def dist(x1, y1, x2, y2):
    return math.sqrt((y1 - y2)**2 + (x1 - x2)**2)
def clamp(a, min_, max_):
    if a < min_: return min_
    if a > max_: return max_
    return a

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
            'id': int(self.robot.getName()[1]) - 1
        })
        ###################################### Daryaft Data az team
        while self.is_new_team_data():
            team_data = self.get_new_team_data()['robot_id']
            if not self.is_ball and team_data['is_ball']:
                self.xb = team_data['xb']
                self.yb = team_data['yb']
                self.is_ball = True
            self.robotposes[team_data['id']] = [
                team_data['xr'], 
                team_data['yr'], 
                dist(team_data['xr'], team_data['yr'], self.xb, self.yb)
            ]
        self.robotposes[int(self.robot.getName()[1]) - 1] = [self.xr, self.yr, self.ball_distance]
        ###################################### Peyda kardane doortarin robot
        doortarin = self.robotposes[0][2]
        doortarinID = 1
        nazdiktarin = self.robotposes[0][2]
        nazdiktarinID = 1
        for i in range(3):
            if self.robotposes[i][2] > doortarin:
                doortarin = self.robotposes[i][2]
                doortarinID = i + 1
            if self.robotposes[i][2] < nazdiktarin:
                nazdiktarin = self.robotposes[i][2]
                nazdiktarinID = i + 1
        ###################################### Moshakhas kardane naghshe robot
        if int(self.robot.getName()[1]) == doortarinID:
            self.role = 'GoalKeeper'
        else:
            self.role = 'Forward'
            if int(self.robot.getName()[1]) != nazdiktarinID:
                self.is_defence_robot = True
            else:
                self.is_defence_robot = False
        ###################################### Tashkhis Harekate toop va robot va penalty area time
        step = 1
        threshold = 0.1
        if time.time() - self.start_time > step:
            if dist(self.last_xb, self.last_yb, self.xb, self.yb) < threshold:
                self.ball_stop_time += step
            else:
                self.ball_stop_time = 0
            if dist(self.last_xr, self.last_yr, self.xr, self.yr) < threshold:
                self.robot_stop_time += step
            else:
                self.robot_stop_time = 0
            if self.xr > -0.35 and self.xr < 0.35 and self.yr < -0.59:
                self.penalty_area_time += step
            else:
                self.penalty_area_time = 0
            self.last_xb = self.xb
            self.last_yb = self.yb
            self.last_xr = self.xr
            self.last_yr = self.yr
            self.start_time = time.time()
    def get_nearest_neutral_spot(self):
        ALL_NEUTRAL_SPOTS = [
            [ 0   ,  0   ,  0    , -0.15],
            [-0.3 , -0.3 , -0.38 , -0.42],
            [ 0   , -0.2 ,  0    , -0.35],
            [ 0.3 , -0.3 ,  0.38 , -0.42],
            [ 0.3 ,  0.3 ,  0.36 ,  0.15],
            [ 0   ,  0.2 ,  0    ,  0.5 ],
            [-0.3 ,  0.3 , -0.36 ,  0.15],
        ]
        NEUTRAL_SPOTS = []
        for i in range(len(ALL_NEUTRAL_SPOTS)):
            for j in range(3):
                if dist(self.robotposes[j][0], self.robotposes[j][1], ALL_NEUTRAL_SPOTS[i][0], ALL_NEUTRAL_SPOTS[i][1]) > 0.08:
                    NEUTRAL_SPOTS.append(ALL_NEUTRAL_SPOTS[i])
        min_dist = dist(self.xb, self.yb, NEUTRAL_SPOTS[0][0], NEUTRAL_SPOTS[0][1])
        min_index = 0
        for i in range(len(NEUTRAL_SPOTS)):
            if dist(self.xb, self.yb, NEUTRAL_SPOTS[i][0], NEUTRAL_SPOTS[i][1]) < min_dist:
                min_dist = dist(self.xb, self.yb, NEUTRAL_SPOTS[i][0], NEUTRAL_SPOTS[i][1])
                min_index = i
        return NEUTRAL_SPOTS[min_index]
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
    def moveAndLook(self, x, y, x1, y1):
        angle = math.degrees(math.atan2(self.xr - x, y - self.yr))
        dest_angle = self.heading - angle
        if dest_angle > 180: dest_angle -= 360
        if dest_angle <-180: dest_angle += 360

        if dist(self.xr, self.yr, x, y) < 0.02:
            angle = math.degrees(math.atan2(self.xr - x1, y1 - self.yr))
            dest_angle = self.heading - angle
            if dest_angle > 180: dest_angle -= 360
            if dest_angle <-180: dest_angle += 360
            self.motor(-dest_angle*0.4, dest_angle*0.4)
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
            self.moveAndLook(0, -0.6, 0, 0.75)
        if self.robot.getName()[1] == '2':
            self.moveAndLook(-0.3, -0.2, 0, 0.75)
        if self.robot.getName()[1] == '3':
            self.moveAndLook(0.3, -0.2, 0, 0.75)
    def GoalKeeperAI(self):
        if self.yb < -0.5 and (self.xb > 0.3 or self.yb < -0.3):
            if self.xb > 0:
                self.move(0.4, self.yb)
            else:
                self.move(-0.4, self.yb)
        elif self.yr > -0.5 or self.yr < -0.55:
            self.move(clamp(self.xb, -0.4, 0.4), -0.55)
        else:
            if self.heading > 95: 
                self.motor(-10, 10)
            elif self.heading < 85:
                self.motor(10, -10)
            else:
                if clamp(self.xb, -0.4, 0.4) > self.xr:
                    self.motor(-10, -10)
                else:
                    self.motor(10, 10)
    def PenaltyAreaTimeout(self):
        self.goalkeeper_running = True
        self.move(self.xb, 0)
        if(self.yr > -0.2):
            self.goalkeeper_running = False
        # self.ForwardAI()
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
        self.robotposes = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
        self.role = 'Forward'
        self.start_time = time.time()
        self.last_xb = 0
        self.last_yb = 0
        self.last_xr = 0
        self.last_yr = 0
        self.ball_stop_time = 0
        self.robot_stop_time = 0
        self.penalty_area_time = 0
        self.goalkeeper_running = False
        self.is_defence_robot = False
        while self.robot.step(TIME_STEP) != -1:
            self.readData()
            self.GoalKeeperAI()
           
            