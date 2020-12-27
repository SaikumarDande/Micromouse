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
from numpy import asarray
from numpy import save
from std_msgs.msg import Empty
from std_srvs.srv import Empty as emt

MAX_SIZE = 9223372036854775807

robot_location = [-8, 8]  #[7, -7] #0 : x, 1: y

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def clbk_laser(msg):
    global regions          #360 lines
    regions = {
        'right': min(min(msg.ranges[0:10]), 10),
        'front': min(min(msg.ranges[174:185]), 10),
        'left':min(min(msg.ranges[349:360]), 10)
    }

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
        data.angular.z = 5*angle
        if abs(angle1)>math.pi/2:
            data.linear.x = -0.6
        else:
            data.linear.x = 0.6
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

    Kp =  8     #5
    Ki =  0.002      #0
    Kd =  4      #0.5
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


class gridNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        #True means no wall is present on that side, and False means wall is present
        self.walls = [True, True, True, True] #E, N, W, S -> 0, 1, 2, 3 -> Right, Up, Left, Down

class FloodFillAlgo:
    def __init__(self, start, goal, algo):
        self.n = 16
        self.m = 16
        self.start = start
        self.goal = goal
        self.floodFill = [[-1]*self.m for _ in range(self.n)]
        self.exploredMaze = None
        self.setBoundariesOfMaze()
        self.path = []
        self.reachedGoal = None
        self.direction = 3      #E:0, N:1, W:2, S:3
        self.d = {(1, 0): 0, (0, 1):1, (-1, 0):2, (0, -1):3}
        self.nd = {0:(0, 1), 1:(-1, 0), 2:(0, -1), 3:(1, 0)}
        self.algo = algo        # 1: 0, 2:-1, 3:random
    
    def setBoundariesOfMaze(self):
        self.exploredMaze = [[0]*self.m for _ in range(self.n)]
        for i in range(self.n):
            for j in range(self.m):
                new_node = gridNode(i, j)
                if i == 0:
                    new_node.walls[1] = False
                if i == self.n-1:
                    new_node.walls[3] = False
                if j == 0:
                    new_node.walls[2] = False
                if j == self.m-1:
                    new_node.walls[0] = False
                self.exploredMaze[i][j] = new_node
    
    def setCurrentNodeBoundaries(self, node):
        wallAdded = False

        if self.exploredMaze[node[0]][node[1]].walls[self.direction] and regions['front']<0.18:
            wallAdded = True
            self.exploredMaze[node[0]][node[1]].walls[self.direction] = False
            next_r = node[0] + self.nd[self.direction][0]
            next_c = node[1] + self.nd[self.direction][1]
            if 0<=next_r<=16 and 0<=next_c<=16:
                self.exploredMaze[next_r][next_c].walls[(self.direction+2)%4] = False
        
        if self.exploredMaze[node[0]][node[1]].walls[(self.direction-1)%4] and regions['right']<0.18:
            wallAdded = True
            self.exploredMaze[node[0]][node[1]].walls[(self.direction-1)%4] = False
            next_r = node[0] + self.nd[(self.direction-1)%4][0]
            next_c = node[1] + self.nd[(self.direction-1)%4][1]
            if 0<=next_r<=16 and 0<=next_c<=16:
                self.exploredMaze[next_r][next_c].walls[(self.direction+1)%4] = False
        
        if self.exploredMaze[node[0]][node[1]].walls[(self.direction+1)%4] and regions['left']<0.18:
            wallAdded = True
            self.exploredMaze[node[0]][node[1]].walls[(self.direction+1)%4] = False
            next_r = node[0] + self.nd[(self.direction+1)%4][0]
            next_c = node[1] + self.nd[(self.direction+1)%4][1]
            if 0<=next_r<=16 and 0<=next_c<=16:
                self.exploredMaze[next_r][next_c].walls[(self.direction-1)%4] = False
        
        return wallAdded

    def get_grid(self):
        return self.exploredMaze
    
    def fillGrid(self):
        start = self.start
        stack = []
        for g in self.goal:
            self.floodFill[g[0]][g[1]] = 0
            stack.append(g)
        
        while(len(stack)>0):
            curr = stack.pop(0)
            up = self.exploredMaze[curr[0]][curr[1]].walls[1]
            down = self.exploredMaze[curr[0]][curr[1]].walls[3]
            left = self.exploredMaze[curr[0]][curr[1]].walls[2]
            right = self.exploredMaze[curr[0]][curr[1]].walls[0]

            #Straight paths here
            if up == True and self.floodFill[curr[0]-1][curr[1]] == -1:
                self.floodFill[curr[0]-1][curr[1]] = 1+self.floodFill[curr[0]][curr[1]]
                stack.append([curr[0]-1, curr[1]])
            if down == True and self.floodFill[curr[0]+1][curr[1]] == -1:
                self.floodFill[curr[0]+1][curr[1]] = 1+self.floodFill[curr[0]][curr[1]]
                stack.append([curr[0]+1, curr[1]])
            if left == True and self.floodFill[curr[0]][curr[1]-1] == -1:
                self.floodFill[curr[0]][curr[1]-1] = 1+self.floodFill[curr[0]][curr[1]]
                stack.append([curr[0], curr[1]-1])
            if right == True and self.floodFill[curr[0]][curr[1]+1] == -1:
                self.floodFill[curr[0]][curr[1]+1] = 1+self.floodFill[curr[0]][curr[1]]
                stack.append([curr[0], curr[1]+1])

    def get_path(self):
        path = []
        goalFound = False
        curr = self.start
        nextMinNode = curr
        miniDistance = self.floodFill[curr[0]][curr[1]]
        while not goalFound:
            curr = nextMinNode
            miniDistance = self.floodFill[curr[0]][curr[1]]
            up = self.exploredMaze[curr[0]][curr[1]].walls[1]
            down = self.exploredMaze[curr[0]][curr[1]].walls[3]
            left = self.exploredMaze[curr[0]][curr[1]].walls[2]
            right = self.exploredMaze[curr[0]][curr[1]].walls[0]
            
            possible_dir = []

            if up and self.floodFill[curr[0]-1][curr[1]]<miniDistance:
                possible_dir.append([curr[0]-1, curr[1]])
            
            if down and self.floodFill[curr[0]+1][curr[1]]<miniDistance:
                possible_dir.append([curr[0]+1, curr[1]])
            
            if left and self.floodFill[curr[0]][curr[1]-1]<miniDistance:
                possible_dir.append([curr[0], curr[1]-1])
            
            if right and self.floodFill[curr[0]][curr[1]+1]<miniDistance:
                possible_dir.append([curr[0], curr[1]+1])
            
            if self.algo == 1:
                nextMinNode = possible_dir[0]
            elif self.algo == 2:
                nextMinNode = possible_dir[-1]
            else:
                i = random.randint(0, len(possible_dir)-1)
                nextMinNode = possible_dir[i]
            
            miniDistance = self.floodFill[nextMinNode[0]][nextMinNode[1]]

            for g in self.goal:
                if nextMinNode[0]==g[0] and nextMinNode[1]==g[1]:
                    goalFound = True
            
            path.append(nextMinNode)
        return path

    def findPath(self):
        self.floodFill = [[-1]*self.m for _ in range(self.n)]
        self.fillGrid()
        self.path = self.get_path()

    def StartExploring(self):
        actual_path = [self.start]
        goalFound = False
        for g in self.goal:
            if self.start[0] == g[0] and self.start[1] == g[1]:
                goalFound = True
         
        while not goalFound:
            wallAdded = self.setCurrentNodeBoundaries(self.start)
            if wallAdded or len(self.path)==0:
                self.findPath()

            nextNode = self.path.pop(0)
            prev_direction = self.direction
            self.direction = self.d[(nextNode[1]-self.start[1], self.start[0]-nextNode[0])]
            if prev_direction != self.direction:
                rotate(self.direction)
            move_one_step_forward(self.direction)

            if len(actual_path)>1 and actual_path[-2][0] == nextNode[0] and actual_path[-2][1] == nextNode[1]:
                actual_path.pop()
            else:
                actual_path.append(nextNode)

            self.start = nextNode

            for g in self.goal:
                if self.start[0] == g[0] and self.start[1] == g[1]:
                    goalFound = True
                    self.reachedGoal = g
            
        return actual_path
        
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
#####################################################################################

def start_robot():
    run = True
    start = [0,0]  #[15, 15]
    goal = [[7, 7], [7, 8], [8, 7], [8, 8]] 

    #Initializing flood fill algorithm
    pathFinder = FloodFillAlgo(start, goal, 1)
    start_time = time.time()
    pathFinder.StartExploring()

    #Goal Reached. Going to Starting Position
    print("..............Goal Reached............")
    print()
    print("..............Goining to start position...")

    reachedGoal = pathFinder.reachedGoal
    pathFinder.goal = [start]
    path = pathFinder.StartExploring()
    #Starting Position Reached
    print("..............Reached start position...")
    print()
    print("..............Goining to goal position...")

    pathFinder.goal = [reachedGoal]
    path = pathFinder.StartExploring()
    print("..............Reached goal position...")

    ##Finding the path from start node to goal node and saving it
    pathFinder.start = start
    pathFinder.findPath()

    print("SAVING DATA")
    ##SAVING PATH
    data = asarray(pathFinder.path)
    save('data.npy', data)
    print("SAVED")
    finish_time = time.time()
    print("Time Taken in secs:", finish_time-start_time)
    print("Time Taken in minutes:", (finish_time-start_time)/60)
    
def main():
    global pub
    robot_direction = 3
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub_odom = rospy.Subscriber('/odom', Odometry, odom_callback)
    sub = rospy.Subscriber('/my_mm_robot/laser/scan', LaserScan, clbk_laser)  

    while (True):
        try:
            pose
            regions
            break
        except:
            continue
    
    start_robot()
        
if __name__ == '__main__':
    rospy.init_node('explore', anonymous=True)
    #reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', emt)
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', emt)
    while (True):
        i = int(input("Explore maze(0) or Go to Goal(1): "))
        if i:
            reset_simulation()
            pub1 = rospy.Publisher('gotogoal', Empty, queue_size=1)
            pub1.publish()
        else:
            print("Starting exploring")
            main()
