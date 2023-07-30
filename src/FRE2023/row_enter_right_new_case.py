#! /usr/bin/env python

# import ros stuff
import rospy
from rospy import Duration
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations
from std_msgs.msg import Int32
import math,time
import matplotlib
matplotlib.use('GTK3Agg')
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
yaw_precision_ = (math.pi /180)*4 # +/- 2 degree allowed
bp=[[0.5,0.100],[2.0,0.25],[0.5,-0.25],[2.0,-0.100]]#,[0.15,0.1],[0.5,0.2],[0.15,-0.2],[0.5,-0.1]]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2= bp[3][0]-bp[2][0]
height2= bp[3][1]-bp[2][1]
# width3= bp[5][0]-bp[4][0]
# height3=bp[5][1]-bp[4][1]
# width4 = bp[7][0]-bp[6][0]
# height4 = bp[7][1]-bp[6][1]
dist_precision_ = 0.1
right_dist=[]
left_dist=[]
right_dist_avg = 0
left_dist_avg = 0
# publishers
pub = None
desired_orientation_=0
switch=0
case_=2
point_=0
first_row_y= 0.45
row_width = 0.80
right_x_lower_limit=9.9 #5 #9.2
right_x_higher_limit=10.6
laser_ranges = []
laser_angles = []
# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    global active_,right_x_lower_limit
    
    # position
    position_ = msg.pose.pose.position
    if position_.x>=right_x_lower_limit:
        active_= True
    elif position_.y<right_x_lower_limit:
        active_=False
        # change_state(0)
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    
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
        yaw=math.degrees(yaw_)
        rospy.loginfo("Turtlebot rotates")
        pub.publish(velocity_message)

        # t1 = rospy.Time.now().to_sec()
        # current_angle_degree = (t1-t0)*angular_speed_degree
        
        print('yaw: ',yaw)
        print("yaw offset",yaw_offset)
        print("relative angle degree: ",relative_angle_degree)
        print("(relative_angle_degree-angle_offset+yaw_offset): ",math.fabs(relative_angle_degree-angle_offset+yaw_offset) )
        if point_==0:
            #if (clockwise==1 and yaw_offset<0) or (clockwise==0 and yaw_offset>0):
            if math.fabs(math.radians(yaw))<=math.fabs(normalize_angle(math.radians(relative_angle_degree-angle_offset+yaw_offset))):
                print("(relative_angle_degree-angle_offset+yaw_offset): ",math.fabs(relative_angle_degree-angle_offset+yaw_offset) )
                print("yaw :",math.fabs(yaw))
                break
        elif point_==1 or point_==2:
            #always anticlockwise
            if math.fabs(math.radians(yaw))>=math.fabs(normalize_angle(math.radians(relative_angle_degree-angle_offset+yaw_offset))):
                break
        
        loop_rate.sleep()
                     
        # if  (current_angle_degree>relative_angle_degree):
        #     rospy.loginfo("reached")
        #     break

        

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    pub.publish(velocity_message)

def setDesiredOrientation(speed_in_degree, desired_angle_degree):
    global yaw_
    relative_angle_radians = normalize_angle(math.radians(desired_angle_degree) - yaw_)
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_degree: ",math.degrees(relative_angle_radians))
    print ("desired_angle_degree: ",desired_angle_degree)
    print("clockwise: ",clockwise)
    rotate(speed_in_degree,math.degrees(abs(relative_angle_radians)), clockwise)


def calculate_desired_y():
    global switch,row_width,first_row_y
    desired_y= first_row_y+2*switch*row_width
    return desired_y

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
        # print("number of points detected",X.shape[0])
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

    # print('number of points detected in left box',X_left.shape[0])
    # print('number of points detected in right box',X_right.shape[0])
    
    # print('length of m = ',len(lines_m))   
    # plt.clf()
    # plt.ylim([-0.8, 0.8])
    # plt.xlim([-0, 2.5])
    # plt.grid()
    # plt.title("Box for row-enter-right")
    # # plt.show()
    # rectangle1 = Rectangle(bp[0], width1, height1, linewidth=1, edgecolor='r', facecolor='none')
    # rectangle2 = Rectangle(bp[2], width2, height2, linewidth=1, edgecolor='b', facecolor='none')
    # # rectangle3 = Rectangle(bp[4], width3, height3, linewidth=1, edgecolor='g', facecolor='none')
    # # rectangle4 = Rectangle(bp[6], width4, height4, linewidth=1, edgecolor='m', facecolor='none')
    # ax = plt.gca()
    # ax.add_patch(rectangle1)
    # ax.add_patch(rectangle2)
    # # ax.add_patch(rectangle3)
    # # ax.add_patch(rectangle4)
    # # plt.plot(X, y,'bx')
    # if active_:
    #     plt.scatter(X_left, y_left)
    #     plt.scatter(X_right, y_right)
    #     # plt.scatter(X_left_1, y_left_1)
    #     # plt.scatter(X_right_1, y_right_1)
    #     plt.draw()
    #     plt.pause(0.001)

def avoid_obstacle():
    global pub
    global yaw_, pub, yaw_precision_, state_, regions, X_left,X_right,X_left_1,X_right_1
    twist_msg = Twist()
    # if X_left_1.shape[0]>0:
    #     while X_left_1.shape[0]>0:
    #         twist_msg.angular.z = -0.25 
    #         print('points found on Left_1')
    #         pub.publish(twist_msg)
    #     done(.4)
    if X_left.shape[0]==0 and X_right.shape[0]==0:
        twist_msg.linear.x=0.25
        pub.publish(twist_msg)
    elif X_left.shape[0]>0:
        while X_left.shape[0]>0:
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = -0.15 
            print('points found on Left')
            pub.publish(twist_msg)
        done(0)#

    # if X_right_1.shape[0]>0:
    #     while X_right_1.shape[0]>0:
    #         twist_msg.angular.z = 0.25
    #         print('points found on Right_1')
    #         pub.publish(twist_msg)
    #     done(.4)
    elif X_right.shape[0]>0:
        while X_right.shape[0]>0:
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.15
            print('points found on Right')
            pub.publish(twist_msg)
        done(0)
    
    # done(0.3)      

def change_state(state):
    global state_
    state_ = state
    # print ('State changed to [%s]' % state_)

def change_point(point):
    global point_
    point_ = point

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
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
    
    pub.publish(twist_msg)
    
    # state change conditions
    # if math.fabs(err_yaw) <= yaw_precision_:
    #     # print ('Yaw error: [%s]' % err_yaw)
    #     # change_state(1)
    #     change_state(2)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    print("current goal is x{} & y{}".format(des_pos.x,des_pos.y))
    err_yaw =normalize_angle( desired_yaw - yaw_)
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if math.fabs(err_yaw) > yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        # change_state(1)
        fix_yaw(desired_position)
    elif err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x =0.10 + .25*err_pos
        if err_yaw > 0.03:
                twist_msg.angular.z = 0.1 
        elif err_yaw< -0.03:
            twist_msg.angular.z = -0.1 
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    


def done(desired_vel):
    twist_msg = Twist()
    twist_msg.linear.x = desired_vel
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, position_,active_,desired_orientation_,switch,row_width,right_x_higher_limit,right_x_lower_limit,desired_position
    
    rospy.init_node('row_enter_right_end',anonymous=True)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # srv = rospy.Service('row_entry_right_end_switch', SetBool, row_entry_right_end_switch)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom,queue_size=1)
    # sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser,queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, front_boxes,queue_size=1)
    # sub_slope_number= rospy.Subscriber('/slope_number',Int32,slope_clbk)
    time.sleep(1)
    rate = rospy.Rate (20)
    while not rospy.is_shutdown():
        if not active_:
                print('not active')
                print("switch",switch)
                continue
        for i in range(3):
            print('active')
            desired_position= Point()
            if i==0 :
                done(0)
                # setDesiredOrientation(20,desired_orientation_)
                desired_position.x = right_x_higher_limit
                # r= math.fabs((desired_position.x-position_.x)/math.cos(yaw_))
                # desired_position.y=position_.y+r*math.sin(yaw_)+calculate_desired_offset()*math.fabs(math.cos(yaw_))
                desired_position.y= position_.y#calculate_desired_y()
            if i == 1 :
                done(0)
                # setDesiredOrientation(15,90+desired_orientation_)
                desired_position.x = right_x_higher_limit
                desired_position.y=position_.y+row_width
            if i==2 :
                done(0)
                # setDesiredOrientation(15,180+desired_orientation_)
                desired_position.x = right_x_lower_limit-0.2
                desired_position.y=position_.y
                # r= math.fabs((position_.x-desired_position.x)/math.cos(yaw_))
                # desired_position.y=position_.y+r*math.sin(yaw_)

            while not rospy.is_shutdown():
                if state_ == 0:
                    # fix_yaw(desired_position)
                    if i ==0:
                        change_point(i)
                        setDesiredOrientation(15,desired_orientation_)
                    if i ==1:
                        change_point(i)
                        setDesiredOrientation(15,90+desired_orientation_)
                    if i ==2:
                        change_point(i)
                        setDesiredOrientation(15,180+desired_orientation_)
                    done(0)
                    change_state(1)
                    # print(i)
                    # print('state is 0')
                elif state_ == 1:
                    if i!=2:
                        go_straight_ahead(desired_position)
                    if i==2:
                        if position_.x>right_x_lower_limit:
                            avoid_obstacle()
                        else:
                            change_state(2)
                    # print(i)
                    # print('state is 1')
                elif state_ == 2:
                    # print(i)
                    # print('state is 2')
                    if i!=2:
                        done(0)
                        change_state(0)
                    else:                                                                                                    
                        done(0)
                        switch=switch+1
                        change_state(0)
                        

                    break
                
                else:
                    rospy.logerr('Unknown state!')
                rate.sleep()
        rospy.loginfo("for loop exit successful")
        
        
        rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()