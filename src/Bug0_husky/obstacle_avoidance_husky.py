#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
pub = None

def clbk_laser(msg):
    ranges=msg.ranges
    ranges = [x for x in ranges if not math.isnan(x)]
    custom_range = ranges[119:600]
    regions = {
        'right':  min(min(custom_range[0:96]), 30),
        'fright': min(min(custom_range[96:192]), 30),
        'front':  min(min(custom_range[192:288]), 30),
        'fleft':  min(min(custom_range[288:384]), 30),
        'left':   min(min(custom_range[384:480]), 30),
    }
    
    take_action(regions)
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    state_description = ''
    
    if regions['front'] > 2 and regions['fleft'] > 2 and regions['fright'] > 2:
        state_description = 'case 2 - nothing'
        linear_x = 0.6
        angular_z = 0
    elif regions['front'] < 2 and regions['fleft'] > 2 and regions['fright'] > 2:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 2 and regions['fleft'] > 2 and regions['fright'] < 2:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 2 and regions['fleft'] < 2 and regions['fright'] > 2:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 2 and regions['fleft'] > 2 and regions['fright'] < 2:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < 2 and regions['fleft'] < 2 and regions['fright'] > 2:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < 2 and regions['fleft'] < 2 and regions['fright'] < 2:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > 2 and regions['fleft'] < 2 and regions['fright'] < 2:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def main():
    global pub
    
    rospy.init_node('reading_laser')
    
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=2)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
