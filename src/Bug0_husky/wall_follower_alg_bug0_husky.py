#! /usr/bin/env python

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

active_ = False
pub_ = None
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def wall_follower_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def clbk_laser(msg):
    global regions_
    ranges=msg.ranges
    ranges = [x for x in ranges if not math.isnan(x)]
    custom_range = ranges[119:600]
    regions_ = {
        'right':  min(min(custom_range[0:96]), 30),
        'fright': min(min(custom_range[96:192]), 30),
        'front':  min(min(custom_range[192:288]), 30),
        'fleft':  min(min(custom_range[288:384]), 30),
        'left':   min(min(custom_range[384:480]), 30),
    }

    
    take_action()

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action():
    
    state_description = ''
    
    d = 1.5
    
    if regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions_['front'] > d and regions_['fleft'] > d and regions_['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions_['front'] > d and regions_['fleft'] < d and regions_['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions_['front'] < d and regions_['fleft'] > d and regions_['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions_['front'] < d and regions_['fleft'] < d and regions_['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions_['front'] < d and regions_['fleft'] < d and regions_['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions_['front'] > d and regions_['fleft'] < d and regions_['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(state_description)


def find_wall():
    msg = Twist()
    msg.linear.x = 0.7
    msg.angular.z = -0.3
    return msg

def turn_right():
    msg = Twist()
    msg.angular.z = -0.3
    return msg

def follow_the_wall():
    #global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def main():
    global pub_, active_
    
    rospy.init_node('reading_laser')
    
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    srv = rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_right()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()