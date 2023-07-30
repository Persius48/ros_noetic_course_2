#! /usr/bin/env python

# import ros stuff
import rospy
from rospy import Duration
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations
import math

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0          #0 means fix yaw, 1 means going to point,2 means reached point
#goal pont variable
goal = Point()
goal_received = False #setting the flag variable
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.1
# publishers
pub = None
# callbacks
regions = {}

def clbk_laser(msg):
    global regions
    ranges=msg.ranges
    ranges = [x for x in ranges if not math.isnan(x)]
    custom_range = ranges[270:360]+ranges[0:90]
    regions = {
        # 'right':  min(min(custom_range[0:36]), 10),
        'fright': min(min(custom_range[30:70]), 10),
        'front':  min(min(custom_range[70:110]), 10),
        'fleft':  min(min(custom_range[110:150]), 10),
        # 'left':   min(min(custom_range[144:180]), 10),
    }
    # print(custom_range[110:150])
    print("Front-Right: ",regions['fright'])
    print("Front: ",regions['front'])
    print("Front-Left",regions['fleft'])

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

def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def goal_callback(msg):
    global goal, goal_received
    if not goal_received:
        goal = msg
        goal_received = True
        rospy.loginfo("New goal received: (%f, %f)", goal.x, goal.y)
    else:
        rospy.logwarn("Ignoring new goal, robot is already moving to a goal.")
        rospy.logwarn("Current goal is (%f, %f)", goal.x, goal.y)

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
    if math.fabs(err_yaw) <= yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_, regions
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    twist_msg = Twist()
    fixed_duration = Duration.from_sec(1.5)
    

    if err_pos > dist_precision_:
        if regions['fright']< 0.3:
            while regions['fright']<=0.4:
                twist_msg.angular.z = 0.2 
                print('close to the front-right')
                pub.publish(twist_msg)
            twist_msg.angular.z = 0.0 
            twist_msg.linear.x= .1
            start_time = rospy.Time.now()
            while rospy.Time.now()-start_time < fixed_duration:
                pub.publish(twist_msg)
            twist_msg.linear.x= 0
            pub.publish(twist_msg)
            change_state(2)
        elif regions['fleft']<0.3:
            while regions['fleft']<=0.4:
                twist_msg.angular.z = -0.2 
                pub.publish(twist_msg)
                print('close to the front-left')
            twist_msg.angular.z = 0.0 
            twist_msg.linear.x= .1
            start_time = rospy.Time.now()
            while rospy.Time.now()-start_time < fixed_duration:
                pub.publish(twist_msg)
            twist_msg.linear.x= 0
            pub.publish(twist_msg)
            change_state(2)
        elif regions['front']<.35:
            if regions['fright']<regions['fleft'] or regions['fright']==10:
                while regions['front']<= .50:
                    twist_msg.angular.z = 0.2 
                    print('close to the front')
                    pub.publish(twist_msg)
                twist_msg.angular.z = 0.0
                pub.publish(twist_msg)
                change_state(2)
            elif regions['fright']>regions['fleft'] or regions['fleft']==10:
                while regions['front']<= .50:
                    twist_msg.angular.z = -0.2 
                    print('close to the front')
                    pub.publish(twist_msg)
                twist_msg.angular.z = 0.0
                pub.publish(twist_msg)
                change_state(2)
        else:
            twist_msg.linear.x = 0.2
            if err_yaw > 0.01:
                twist_msg.angular.z = 0.2 
            elif err_yaw< -0.01:
                twist_msg.angular.z = -0.2 
            pub.publish(twist_msg)
            print('going forward')
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, goal_received
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    mid_point_goal_sub=rospy.Subscriber('/mid_point_goal', Point, goal_callback,queue_size=3)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if state_ == 0:
            fix_yaw(goal)
        elif state_ == 1:
            go_straight_ahead(goal)
        elif state_ == 2:
            done()
            goal_received = False
            change_state(0)
            print("done and flag changed to false")
        else:
            rospy.logerr('Unknown state!')
    
        rate.sleep()

    rospy.loginfo("loop exit successful")

if __name__ == '__main__':
    main()