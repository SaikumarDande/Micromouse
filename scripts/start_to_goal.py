#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
import math
import random
from numpy import load
from std_msgs.msg import Empty
from std_srvs.srv import Empty as emt


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def get_distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

def move_one_step_forward(direction):
    d = {0:[1, 0], 1:[0, 1], 2:[-1, 0], 3:[0, -1]}
    tolerance = 0.01
    temp_x = robot_location[0] + d[direction][0]
    temp_y = robot_location[1] + d[direction][1]
    robot_location[0] = temp_x
    robot_location[1] = temp_y
    if temp_x>=0:
        temp_x += 1
    if temp_y<=0:
        temp_y -= 1
    goal_x = (2*abs(temp_x)-1)*0.09
    goal_y = (2*abs(temp_y)-1)*0.09
    if temp_x<0:
        goal_x = -goal_x
    if temp_y<0:
        goal_y = -goal_y
    data = Twist()
    data.linear.y = 0
    data.linear.z = 0
    data.angular.x = 0
    data.angular.y = 0
    rate = rospy.Rate(50)
    angle1 = norm(math.atan2(goal_y-pose[1], goal_x-pose[0])-(pose[2]-math.pi/2))
    angle2 = norm(math.atan2(pose[1]-goal_y, pose[0]-goal_x)-(pose[2]-math.pi/2))
    while(get_distance(pose[0], pose[1], goal_x, goal_y)>tolerance):
        angle1 = norm(math.atan2(goal_y-pose[1], goal_x-pose[0])-(pose[2]-math.pi/2))
        angle2 = norm(math.atan2(pose[1]-goal_y, pose[0]-goal_x)-(pose[2]-math.pi/2))
        if abs(angle1) <abs(angle2):
            angle = angle1
        else:
            angle = angle2
        data.angular.z = 3.5*angle
        if abs(angle1)>math.pi/2:
            data.linear.x = -0.8
        else:
            data.linear.x = 0.8
        pub.publish(data)
        rate.sleep()
    data.linear.x = 0
    data.angular.z = 0
    pub.publish(data)

def rotate(direction):
    tolerance = 0.1
    data = Twist()
    data.linear.x = 0
    data.linear.y = 0
    data.linear.z = 0
    data.angular.x = 0
    data.angular.y = 0
    rate = rospy.Rate(50)
    goal_angle = norm(direction*math.pi/2)

    lastTime = 0
    sample_time = 0.1   #1/rate
    last_yaw = pose[2]
    maxAngle = 5

    Kp =  6     #5
    Ki =  0.002      #0
    Kd =  3.5      #0.5
    errSum = 0
    while(abs(norm(goal_angle - pose[2]+math.pi/2))>tolerance):
        now = int(round(time.time()*1000))
        timeChange = now - lastTime
        if timeChange>=sample_time :
            error = norm(goal_angle - pose[2]+math.pi/2)
            if abs(error)>tolerance:
                if timeChange<100000:
                    errSum += error*timeChange*0.001
                    
                dErr = (pose[2] - last_yaw)/(timeChange*0.01)
                
                out_error= Kp*error + Ki*errSum - Kd*dErr
                last_yaw = pose[2]
                if out_error>maxAngle:
                    out_error = maxAngle
                elif out_error<-1*maxAngle:
                    out_error = -1*maxAngle
                data.angular.z = out_error
            pub.publish(data)
        lastTime = now
        rate.sleep()
    data.angular.z = 0
    pub.publish(data)
    return direction

def norm(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


class goToGoal:
    def __init__(self, start):
        self.start = start
        self.path = []
        self.direction = 3      #E:0, N:1, W:2, S:3
        self.d = {(1, 0): 0, (0, 1):1, (-1, 0):2, (0, -1):3}
        
    def go_to_goal(self):
        while len(self.path):
            nextNode = self.path.pop(0)
            prev_direction = self.direction
            self.direction = self.d[(nextNode[1]-self.start[1], self.start[0]-nextNode[0])]
            if prev_direction != self.direction:
                rotate(self.direction)
            move_one_step_forward(self.direction)
            self.start = nextNode

#####################################################################################

def start_robot():
    path = load('data.npy')
    path = path.tolist()
    start = [0, 0]  # [15, 15]
    pathFinder = goToGoal(start)

    pathFinder.path = path

    print("Started going to goal")
    start_time = time.time()
    pathFinder.go_to_goal()
    end_time = time.time()
    print("TIME TAKEN:", end_time-start_time)
    #Starting Position Reached

def clb(msg):
    reset_simulation()
    global pub, robot_location
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    print("Started go to goal node")
    
    robot_direction = 3
    robot_location = [-8, 8]    #[7, -7]
    while (True):
        try:
            pose
            break
        except:
            continue
    start_robot()
        
if __name__ == '__main__':
    rospy.init_node('gotogoal', anonymous=True)
    global reset_simulation
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', emt)
    sub1 = rospy.Subscriber('/gotogoal', Empty, clb)
    rospy.spin()
    #main()
