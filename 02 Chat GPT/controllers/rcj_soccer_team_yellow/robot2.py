from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP
import math, time

OPONENT_GOAL_Y = 1.5

def dist(x1, y1, x2, y2):
    return math.sqrt((y1 - y2)**2 + (x1 - x2)**2)
def clamp(a, min_, max_):
    if a < min_: return min_
    if a > max_: return max_
    return a
def get_angle(xr, yr, xt, yt, heading):
    angle = math.degrees(math.atan2(xr - xt, yt - yr))
    dest_angle = heading - angle
    dest_angle = (dest_angle + 180) % (2 * 180) - 180
    return dest_angle

class MyRobot2(RCJSoccerRobot):
    def run(self):
        Goal_X = 0
        Goal_Y = 0.75
        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                gps = self.get_gps_coordinates()
                xr = gps[0]
                yr = gps[1]
                heading = math.degrees(self.get_compass_heading())
                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    ball_angle = math.degrees(math.atan2(ball_data['direction'][1], ball_data['direction'][0]))
                    ball_distance = abs(0.01666666/(abs(ball_data['direction'][2])/math.sqrt(1 - ball_data['direction'][2]**2)))
                    xb = -math.sin(math.radians(ball_angle + heading)) * ball_distance + xr
                    yb =  math.cos(math.radians(ball_angle + heading)) * ball_distance + yr
                    diff = get_angle(xr, yr, xb, yb, heading)
                    if yr > yb:
                        if self.xr > self.xb:
                            diff += 20
                        else:
                            diff -= 20
                            
                    if ball_distance < 0.08 and yr < yb:
                        dest_angle = get_angle(xr, yr, Goal_X, Goal_Y, heading)

                        if abs(dest_angle) > 10:
                            self.left_motor.setVelocity( -dest_angle * 0.4)
                            self.right_motor.setVelocity( dest_angle * 0.4)
                        else:
                            self.left_motor.setVelocity( 10)
                            self.right_motor.setVelocity( 10)
                    else:
                        linear_speed = 7 + 10 * ball_distance
                        angular_speed = 0.5 * diff

                        self.left_motor.setVelocity(linear_speed - angular_speed)
                        self.right_motor.setVelocity(linear_speed + angular_speed)
