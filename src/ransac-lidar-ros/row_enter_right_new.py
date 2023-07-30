#! /usr/bin/env python

# import ros stuff
import rospy
from rospy import Duration
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations
from std_msgs.msg import Int32
import math
from std_srvs.srv import SetBool,SetBoolResponse
from matplotlib.patches import Rectangle
import numpy as np
import matplotlib.pyplot as plt
active_ = False
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# parameters
yaw_precision_ = (math.pi /180)*2 # +/- 2 degree allowed
bp=[[0.5,0.100],[2.0,0.3],[0.5,-0.3],[2.0,-0.100],[0.15,0.1],[0.5,0.2],[0.15,-0.2],[0.5,-0.1]]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2= bp[3][0]-bp[2][0]
height2= bp[3][1]-bp[2][1]
width3= bp[5][0]-bp[4][0]
height3=bp[5][1]-bp[4][1]
width4 = bp[7][0]-bp[6][0]
height4 = bp[7][1]-bp[6][1]
dist_precision_ = 0.1
right_dist=[]
left_dist=[]
right_dist_avg = 0
left_dist_avg = 0
# publishers
pub = None
desired_orientation_=0
switch=0
first_row_y= 0.45
row_width = 0.8
right_x_lower_limit=9.9 #5 #9.2
right_x_higher_limit=10.4

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    global active_,right_x_lower_limit
    
    # position
    position_ = msg.pose.pose.position
    if position_.x>=right_x_lower_limit:
        active_= True
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    
def clbk_laser(msg):
    global right_dist,right_dist_avg, left_dist,left_dist_avg
    ranges=msg.ranges
    ranges = [x for x in ranges if not math.isnan(x)]
    right_dist = [x for x in ranges[250:290] if x<.85]
    left_dist = [x for x in ranges[70:110] if x<.85]
    if len(right_dist)==0 and len(left_dist)==0:
        print('no data points for distance found')
    elif len(right_dist)==0 and len(left_dist)>0: 
        left_dist_avg = sum(left_dist)/len(left_dist)
        print('Left :',left_dist_avg)
    elif len(right_dist)>0 and len(left_dist)==0:
        right_dist_avg = sum(right_dist)/len(right_dist)
        print('Right :',right_dist_avg)
    elif len(right_dist)>0 and len(left_dist)>0:
        right_dist_avg = sum(right_dist)/len(right_dist)
        left_dist_avg = sum(left_dist)/len(left_dist)
        print('Right :',right_dist_avg)
        print('Left :',left_dist_avg)
    
# def rotate (angular_speed_degree, relative_angle_degree, clockwise):
#     global pub
    
#     velocity_message = Twist()

#     angular_speed=math.radians(abs(angular_speed_degree))

#     if (clockwise):
#         velocity_message.angular.z =-abs(angular_speed)
#     else:
#         velocity_message.angular.z =abs(angular_speed)

#     angle_moved = 0.0
#     loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
#     #cmd_vel_topic='/turtle1/cmd_vel'
#     #velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

#     t0 = rospy.Time.now().to_sec()

#     while True :
#         rospy.loginfo("Turtlebot rotates")
#         pub.publish(velocity_message)

#         t1 = rospy.Time.now().to_sec()
#         current_angle_degree = (t1-t0)*angular_speed_degree
#         loop_rate.sleep()


                       
#         if  (current_angle_degree>relative_angle_degree):
#             rospy.loginfo("reached")
#             break

#     #finally, stop the robot when the distance is moved
#     velocity_message.angular.z =0
#     pub.publish(velocity_message)
def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    global pub,yaw_
    
    velocity_message = Twist()
    

    angular_speed=math.radians(abs(angular_speed_degree))

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(20) # we publish the velocity at 10 Hz (10 times a second)    
    #cmd_vel_topic='/turtle1/cmd_vel'
    #velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()
    if clockwise:
        relative_angle_degree= -relative_angle_degree
        angle_offset=-2
    else:
        relative_angle_degree= relative_angle_degree
        angle_offset=2
    yaw_offset=math.degrees(yaw_)
    while True :
        rospy.loginfo("Turtlebot rotates")
        yaw=math.degrees(yaw_)
        pub.publish(velocity_message)
        loop_rate.sleep() 
        # t1 = rospy.Time.now().to_sec()
        # current_angle_degree = (t1-t0)*angular_speed_degree
        
        print('yaw: ',yaw)
        if math.fabs(yaw)>=math.fabs(relative_angle_degree-angle_offset+yaw_offset):
            break
                     
        # if  (current_angle_degree>relative_angle_degree):
        #     rospy.loginfo("reached")
        #     break

        

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


def calculate_desired_offset():
    global position_, right_dist,left_dist,right_dist_avg,left_dist_avg
    row_width = 0.85
    if  len(right_dist)!=0 and  len(left_dist)!=0:
        offset_from_mid= -(right_dist_avg-(row_width/2))
    elif len(right_dist)==0 and len(left_dist) !=0:
        offset_from_mid=(left_dist_avg-(row_width/2))
    elif len(right_dist)!=0 and  len(left_dist)==0:
        offset_from_mid=-(right_dist_avg-(row_width/2))
    elif len(right_dist)==0 and  len(left_dist)==0:
        offset_from_mid = 0

    return offset_from_mid

def calculate_desired_y():
    global switch,row_width,first_row_y
    desired_y= first_row_y+2*switch*row_width
    return desired_y



def change_state(state):
    global state_
    state_ = state
    # print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    # rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    print("current goal is x{} & y{}".format(des_pos.x,des_pos.y))
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x =0.3 #0.15 + .25*err_pos
        # if err_yaw > 0.02:
        #         twist_msg.angular.z = 0.2 
        # elif err_yaw< -0.02:
        #     twist_msg.angular.z = -0.2 
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done(desired_vel):
    twist_msg = Twist()
    twist_msg.linear.x = desired_vel
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, position_,active_,desired_orientation_,switch,row_width,right_x_higher_limit,right_x_lower_limit
    
    rospy.init_node('row_enter_right_end',anonymous=True)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # srv = rospy.Service('row_entry_right_end_switch', SetBool, row_entry_right_end_switch)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom,queue_size=1)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    # sub_slope_number= rospy.Subscriber('/slope_number',Int32,slope_clbk)
    rate = rospy.Rate (20)
    while not rospy.is_shutdown():
        if not active_:
                print('not active')
                continue
        for i in range(3):
            print('active')
            desired_position= Point()
            if i==0 :
                done(0)
                # setDesiredOrientation(20,desired_orientation_)
                desired_position.x = right_x_higher_limit
                desired_position.y= calculate_desired_y()
            if i == 1 :
                desired_position.x = right_x_higher_limit
                desired_position.y=position_.y+row_width
            if i==2 :
                setDesiredOrientation(15,180+desired_orientation_)
                desired_position.x = right_x_lower_limit-0.1
                # desired_position.y=position_.y
                r= math.fabs((position_.x-desired_position.x)/math.cos(yaw_))
                desired_position.y=position_.y+r*math.sin(yaw_)
            
            rate=rospy.Rate(20)
            while not rospy.is_shutdown():
                if state_ == 0:
                    fix_yaw(desired_position)
                    print(i)
                    # print('state is 0')
                elif state_ == 1:
                    go_straight_ahead(desired_position)
                    print(i)
                    # print('state is 1')
                elif state_ == 2:
                    print(i)
                    # print('state is 2')
                    if i!=2:
                        done(0)
                        change_state(0)
                    else:                                                                                                    
                        done(.3)
                        switch=switch+1
                        change_state(0)
                        active_=False

                    break
                else:
                    rospy.logerr('Unknown state!')
                # if not active_:
                #     break
                rate.sleep()
        rospy.loginfo("for loop exit successful")
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()