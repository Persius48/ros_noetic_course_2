#! /usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rospy import Duration
from tf import transformations

yaw_= 0
vel_pub=None
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    yaw_=math.degrees(yaw_)
    x_vel= msg.twist.twist.linear.x
    y_vel=msg.twist.twist.linear.y
    ang_vel=msg.twist.twist.angular.z

    print('x =',position_.x)
    print('y =',position_.y)
    print('yaw =',yaw_)
    # print('x_vel: ',x_vel)
    # print('y_vel: ',y_vel)
    # print('ang_vel ',ang_vel)


# def clbk_laser(msg):
#     global regions, right_dist,right_dist_avg, left_dist,left_dist_avg,front_dist,front_dist_avg
#     ranges=msg.ranges
#     ranges = [x for x in ranges if not math.isnan(x)]
#     custom_range = ranges[270:360]+ranges[0:90]

#     right_dist = [x for x in custom_range[0:70] if x<.75]
#     left_dist = [x for x in custom_range[110:180] if x<.75]
#     front_dist = [x for x in custom_range[70:110] if x<1]
#     if len(right_dist)==0 and len(left_dist)==0:
#         pass
#         #print('no data points for distance found')
#     elif len(right_dist)==0 and len(left_dist)>0: 
#         left_dist_avg = sum(left_dist)/len(left_dist)
#         right_dist_avg = 0
#         #print('Right :',right_dist_avg)
#         #print('Left :',left_dist_avg)
#     elif len(right_dist)>0 and len(left_dist)==0:
#         right_dist_avg = sum(right_dist)/len(right_dist)
#         left_dist_avg = 0
#         #print('Right :',right_dist_avg)
#         #print('Left :',left_dist_avg)
#     elif len(right_dist)>0 and len(left_dist)>0:
#         right_dist_avg = sum(right_dist)/len(right_dist)
#         left_dist_avg = sum(left_dist)/len(left_dist)
#         #print('Right :',right_dist_avg)
#         #print('Left :',left_dist_avg)

#     if len(front_dist) == 0:
#         pass
#         #print('no data points found in front.')
#     elif len(front_dist) > 0:
#         front_dist_avg = sum(front_dist)/len(front_dist)
#         #print('Front : ',front_dist_avg)


    
#     regions = {
#         'right':  min(min(custom_range[0:70]), 0.75),
#         'right-1': min(min(custom_range[30:70]),0.75),
#         'fright': min(min(custom_range[70:85]), 1),
#         'front':  min(min(custom_range[70:110]), 1),
#         'forfront': min(min(custom_range[85:95]),1),
#         'fleft':  min(min(custom_range[95:110]), 1),
#         'left':   min(min(custom_range[110:180]), 0.75),
#         'left-1': min(min(custom_range[110:150]),0.75),
#     }
    
def rotate (angular_speed_degree, relative_angle_degree, clockwise):
    global vel_pub,yaw_
    
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
    if clockwise:
        relative_angle_degree= -relative_angle_degree
        angle_offset=-2
    else:
        relative_angle_degree= relative_angle_degree
        angle_offset=2

    while True :
        rospy.loginfo("Turtlebot rotates")
        vel_pub.publish(velocity_message)

        # t1 = rospy.Time.now().to_sec()
        # current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()
        print('yaw: ',yaw_)
        if math.fabs(yaw_)>=math.fabs(relative_angle_degree-angle_offset):
            break
                       
        # if  (current_angle_degree>relative_angle_degree):
        #     rospy.loginfo("reached")
        #     break

        

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    vel_pub.publish(velocity_message)
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
def main():
    global vel_pub,yaw_
    rospy.init_node('row_navigation',anonymous=True)
    # sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom,queue_size=1)
    vel_pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    fixed_duration = Duration.from_sec(6)
    twist_msg=Twist()
    # rotate(20,90,clockwise=True)
    setDesiredOrientation(15,180)
    # twist_msg.angular.z= -0.012
    # twist_msg.linear.x= 0.4
    # start_time = rospy.Time.now()
    # rate=rospy.Rate(10)
    # while rospy.Time.now()-start_time < fixed_duration:
    #     vel_pub.publish(twist_msg)
    #     rate.sleep()
    # twist_msg.angular.z=0
    # twist_msg.linear.x=0
    # vel_pub.publish(twist_msg)
    # rospy.spin()

if __name__ == '__main__':
    main()