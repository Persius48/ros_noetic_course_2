#! /usr/bin/env python3

# import ros stuff
import rospy
from rospy import Duration
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations
import matplotlib
matplotlib.use('GTK3Agg')
from std_msgs.msg import Int32
import math
import matplotlib
import time
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool,SetBoolResponse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# robot state variables
position_ = Point()
yaw_ = 0
yaw_imu=0
# machine state
state_ = 0          #0 means fix yaw, 1 means going to point,2 means reached point
#goal pont variable
goal = Point()
goal_received = False #setting the flag variable
# parameters
yaw_precision_ = (math.pi /180)*4  # +/- 2 degree allowed
dist_precision_ = 0.1
# publishers
pub = None
right=[]
front_right=[]
front =[]
front_left=[]
left=[]
regions = {}
active_ = False
n = 0
yaw_case = 1
right_x_lower_limit= 15
left_x_higher_limit = 0.3
laser_ranges = []
laser_angles = []
# X =[]
# y=[]
# X = np.array(X)
# X_left=X
# y_left=y
# X_left_1=X
# y_left_1=y
# X_right=X
# y_right=y
# X_right_1=X
# y_right_1=y
# bp=[[0.5,0.100],[1.0,0.3125],[0.5,-0.3125],[1.0,-0.100]]
#bp=[[0.5,0.100],[2,0.31],[0.5,-0.31],[2,-0.100]]#,[0.15,0.1],[0.5,0.2],[0.15,-0.2],[0.5,-0.1]]
bp=[[0.5,0.100],[1.3,0.27],[0.5,-0.27],[1.3,-0.100]]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2= bp[3][0]-bp[2][0]
height2= bp[3][1]-bp[2][1]
# width3= bp[5][0]-bp[4][0]
# height3=bp[5][1]-bp[4][1]
# width4 = bp[7][0]-bp[6][0]
# height4 = bp[7][1]-bp[6][1]

# def go_to_mid_point_switch(req):
#     global active_
#     active_ = req.data
#     print(f"Received request to set active_ to {active_}")
#     res = SetBoolResponse()
#     res.success = True
#     res.message = 'Done!'
#     return res
  
def clbk_odom(msg):
    global position_
    global yaw_,active_,n,right_x_lower_limit,left_x_higher_limit
    
    # position
    position_ = msg.pose.pose.position
    if n==0:
        if  position_.x<right_x_lower_limit:
            active_= True
        # elif  position_.x>=right_x_lower_limit:
        #     active_= False
        #     n = None

    if n==None:
        if position_.x >left_x_higher_limit and position_.x <right_x_lower_limit:
            active_= True
        # elif position_.x<=left_x_higher_limit or position_.x >= right_x_lower_limit:
        #     active_= False 
    
        # done()
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def front_boxes(msg):
    global laser_ranges,laser_angles,X_right,y_right,X_left,y_left,X_left_1,y_left_1,X_right_1,y_right_1
    laser_ranges = msg.ranges 
    
    for i in range(0,len(laser_ranges)):
       laser_angles.append(msg.angle_min+i*msg.angle_increment)
    # for i in range(0,180):
    #    laser_angles.append(0+i*msg.angle_increment)


    for j in range(0,len(bp),2):
        X = []
        y = []

        for i in range(0, len(laser_ranges)):
            pn_x = laser_ranges[i]*math.cos(laser_angles[i])
            # print(pn_x)
            pn_y = laser_ranges[i]*math.sin(laser_angles[i])
            # print(pn_y)

            if (math.isinf(pn_x)==False and math.isinf(pn_y)==False and bp[j][0]<pn_x<bp[j+1][0] and bp[j][1]<pn_y<bp[j+1][1]):
                    X.append([pn_x])
                    # print(X)
                    y.append([pn_y])
                    # print(y)

            else: 
                pass

        X = np.array(X)
        print("number of points detected",X.shape[0])
        y = np.array(y)
        if j==0:
            X_left=X
            y_left=y
        # print('result = ',result)
        if j==2:
            X_right=X
            y_right=y
        # if j==4:
        #     X_left_1=X
        #     y_left_1=y
        # if j==6:
        #     X_right_1=X
        #     y_right_1=y

    print('number of points detected in left box',X_left.shape[0])
    print('number of points detected in right box',X_right.shape[0])
    
    # print('length of m = ',len(lines_m))   
    plt.clf()
    plt.ylim([-0.8, 0.8])
    plt.xlim([-0, 2.5])
    plt.grid()
    plt.title("Ransac lines on the plot!")
    # plt.show()
    rectangle1 = Rectangle(bp[0], width1, height1, linewidth=1, edgecolor='r', facecolor='none')
    rectangle2 = Rectangle(bp[2], width2, height2, linewidth=1, edgecolor='b', facecolor='none')
    # rectangle3 = Rectangle(bp[4], width3, height3, linewidth=1, edgecolor='g', facecolor='none')
    # rectangle4 = Rectangle(bp[6], width4, height4, linewidth=1, edgecolor='m', facecolor='none')
    ax = plt.gca()
    ax.add_patch(rectangle1)
    ax.add_patch(rectangle2)
    # ax.add_patch(rectangle3)
    # ax.add_patch(rectangle4)
    # plt.plot(X, y,'bx')

    plt.scatter(X_left, y_left)
    plt.scatter(X_right, y_right)
    # plt.scatter(X_left_1, y_left_1)
    # plt.scatter(X_right_1, y_right_1)
    plt.draw()
    plt.pause(0.001) 

def imu_callback(msg):
    global yaw_imu
    orientation = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    euler_angles = transformations.euler_from_quaternion(orientation)

    yaw_imu =euler_angles[2]

def change_state(state):
    global state_
    state_ = state
    # print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    global pub
    
    velocity_message = Twist()

    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    #cmd_vel_topic='/turtle1/cmd_vel'
    #velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Turtlebot rotates")
        pub.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()


                       
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    pub.publish(velocity_message)

def setDesiredOrientation(speed_in_degree, desired_angle_degree):
    global yaw_
    relative_angle_radians = math.radians(desired_angle_degree) - yaw_
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_degree: ",math.degrees(relative_angle_radians))
    print ("desired_angle_degree: ",desired_angle_degree)
    rotate(speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_, X_left,X_right,X_left_1,X_right_1
    twist_msg = Twist()
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(1)
def forward_velocity():
    global pub
    twist_msg=Twist()
    if position_.x<0.5 or position_.x>3.5:
        twist_msg.linear.x=0.8
    else:
        twist_msg.linear.x=0.8
    

    pub.publish(twist_msg)
    # if X_left_1.shape[0]>0:
    #     change_state(1)
    if X_left.shape[0]>0:
        change_state(1)
    # if X_right_1.shape[0]>0:
    #     change_state(1)
    if X_right.shape[0]>0:
        change_state(1)
    # if regions['right-1']< 0.28:
    #     change_state(1)
    # if regions['left-1']< 0.28:
    #     change_state(1) 

def avoid_obstacle():
    global pub
    global yaw_imu, pub, yaw_precision_, state_, regions, X_left,X_right,X_left_1,X_right_1
    twist_msg = Twist()
    rate=rospy.Rate(20)
  
    if X_left.shape[0]>0 and X_right.shape[0]>0:
        # if (yaw_>=-90 and yaw_<=90):
        if yaw_imu>=0 and yaw_imu<=90 or (yaw_imu>=-180 and yaw_imu<=-90):
            while X_left.shape[0]>0:
                twist_msg.linear.x = 0.8
                twist_msg.angular.z = -0.15 
                print('points found on Left')
                pub.publish(twist_msg)
                rate.sleep()
            change_state(0)
        elif yaw_imu>=-90 and yaw_imu<=0 or (yaw_imu>=90 and yaw_imu<=180):
            while X_right.shape[0]>0:
                twist_msg.linear.x = 0.8
                twist_msg.angular.z = 0.15
                print('points found on Right')
                pub.publish(twist_msg)
                rate.sleep()
            change_state(0)

    if X_left.shape[0]>0:
        while X_left.shape[0]>0:
            twist_msg.linear.x = 0.8
            twist_msg.angular.z = -0.15
            print('points found on Left')
            pub.publish(twist_msg)
            rate.sleep()
        change_state(0)

    elif X_right.shape[0]>0:
        while X_right.shape[0]>0:
            twist_msg.linear.x = 0.8
            twist_msg.angular.z = 0.15
            print('points found on Right')
            pub.publish(twist_msg)
            rate.sleep()
        change_state(0)



def done(velocity):
    twist_msg = Twist()
    twist_msg.linear.x = velocity
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    


def main():
    global pub, goal_received,active_,position_,yaw_case,yaw_imu,n
    
    rospy.init_node('row_navigation',anonymous=True)
    # srv = rospy.Service('go_to_mid_point_switch', SetBool, go_to_mid_point_switch)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, front_boxes,queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom,queue_size=1)
    sub_imu = rospy.Subscriber('/imu/data',Imu,imu_callback,queue_size=1)
    # sub_slope_number= rospy.Subscriber('/slope_number',Int32,slope_clbk)
    time.sleep(2)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            print('not active)')
            goal_received = False
            continue
        else:
            if state_ == 0:
                if  position_.x>=right_x_lower_limit:
                    done(0)
                    n=None
                    active_= False
                elif n==None and position_.x<=left_x_higher_limit:
                    done(0)
                    # change_state(0)
                    active_= False
                else:
                    forward_velocity()
                print('state 0')
            elif state_ == 1:
                avoid_obstacle()
                print('state 1')
                if  position_.x>=right_x_lower_limit:
                    done(0)
                    n=None
                    # change_state(0)
                    active_= False
                if n==None and position_.x<=left_x_higher_limit:
                    done(0)
                    # change_state(0)
                    active_= False
                
            # elif state_ == 2:
            #     # print('state 2)')
            #     done()
            #     goal_received = False
            #     change_state(0)
            #     print("done and flag changed to false")
            else:
                rospy.logerr('Unknown state!')
    
            rate.sleep()

    rospy.loginfo("loop exit successful")

if __name__ == '__main__':
    main()