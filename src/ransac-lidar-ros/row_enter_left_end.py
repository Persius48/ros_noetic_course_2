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

active_ = False
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.1

# publishers
pub = None
m_count_=0
n=0

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    global active_
    global n
    
    # position
    position_ = msg.pose.pose.position
    if n==0:
        active_= False
        if position_.x>1:
            n=None
    elif n==None:
        if position_.x<=1:
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
    # print("Front-Right: ",regions['fright'])
    # print("Front: ",regions['front'])
    # print("Front-Left",regions['fleft'])



# def row_entry_right_end_switch(req):
#     global active_
#     active_ = req.data
#     print(f"Received request to set active_ to {active_}")
#     res = SetBoolResponse()
#     res.success = True
#     res.message = 'Done!'
#     if not active_:
#         done()  # stop the robot if the service is called with False
#     return res
# def slope_clbk(msg):
#     global position_,active_,m_count_
#     max_m_count= 5
#     if position_.x<9.5:
#         active_= False
#     elif position_.x>=9.5:
#         active_= True
        # m_count_ +=1
        # if m_count_>max_m_count:
        #     active_= True
        #     m_count_=0




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
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    
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
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        # print ('Yaw error: [%s]' % err_yaw)
        change_state(0)
# def go_straight_ahead(des_pos):
#     global yaw_, pub, yaw_precision_, state_, regions
#     desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
#     print("current goal is x{} & y{}".format(des_pos.x,des_pos.y))
#     err_yaw = desired_yaw - yaw_
#     err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
#     twist_msg = Twist()
#     fixed_duration = Duration.from_sec(.5)
    

#     if err_pos > dist_precision_:
#         if regions['fright']< 0.3:
#             while regions['fright']<=0.4:
#                 twist_msg.angular.z = 0.2 
#                 print('close to the front-right')
#                 pub.publish(twist_msg)
#             twist_msg.angular.z = 0.0 
#             twist_msg.linear.x= .1
#             start_time = rospy.Time.now()
#             while rospy.Time.now()-start_time < fixed_duration:
#                 pub.publish(twist_msg)
#             twist_msg.linear.x= 0
#             pub.publish(twist_msg)
#             change_state(1)

#         elif regions['fleft']<0.3:
#             while regions['fleft']<=0.4:
#                 twist_msg.angular.z = -0.2 
#                 pub.publish(twist_msg)
#                 print('close to the front-left')
#             twist_msg.angular.z = 0.0 
#             twist_msg.linear.x= .1
#             start_time = rospy.Time.now()
#             while rospy.Time.now()-start_time < fixed_duration:
#                 pub.publish(twist_msg)
#             twist_msg.linear.x= 0
#             pub.publish(twist_msg)
#             change_state(1)
#         elif regions['front']<.45:
#             if regions['fright']<regions['fleft'] or regions['fright']==10:
#                 while regions['front']<= 1:
#                     twist_msg.angular.z = 0.2 
#                     print('close to the front')
#                     pub.publish(twist_msg)
#                 twist_msg.angular.z = 0.0
#                 twist_msg.linear.x= .2
#                 start_time = rospy.Time.now()
#                 while rospy.Time.now()-start_time < fixed_duration:
#                     pub.publish(twist_msg)
#                 twist_msg.linear.x= 0
#                 pub.publish(twist_msg)
#                 change_state(1)
#             elif regions['fright']>regions['fleft'] or regions['fleft']==10:
#                 while regions['front']<= 1:
#                     twist_msg.angular.z = -0.2 
#                 twist_msg.angular.z = 0.0
#                 twist_msg.linear.x= .2
#                 start_time = rospy.Time.now()
#                 while rospy.Time.now()-start_time < fixed_duration:
#                     pub.publish(twist_msg)
#                 twist_msg.linear.x= 0
#                 pub.publish(twist_msg)
#                 change_state(1)
#         else:
#             twist_msg.linear.x = 0.25
#             if err_yaw > 0.02:
#                 twist_msg.angular.z = 0.2 
#             elif err_yaw< -0.02:
#                 twist_msg.angular.z = -0.2 
#             pub.publish(twist_msg)

#     else:
#         print ('Position error: [%s]' % err_pos)
#         change_state(2)
    
#     state change conditions
#     if math.fabs(err_yaw) > yaw_precision_:
#         print ('Yaw error: [%s]' % err_yaw)
#         change_state(0)


def done(desired_vel):
    twist_msg = Twist()
    twist_msg.linear.x = desired_vel
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, position_,active_
    
    rospy.init_node('row_enter_right_end',anonymous=True)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # srv = rospy.Service('row_entry_right_end_switch', SetBool, row_entry_right_end_switch)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    # sub_slope_number= rospy.Subscriber('/slope_number',Int32,slope_clbk)
    rate = rospy.Rate (10)
    while not rospy.is_shutdown():

        if not active_:
                print('not active')
                continue
        for i in range(3):
            print('active')
            desired_position= Point()
            if i==0 :
                desired_position.x = -1 
                desired_position.y=position_.y
            if i == 1 :
                desired_position.x = -1
                desired_position.y=position_.y+.9
            if i==2 :
                desired_position.x = 1.1
                desired_position.y=position_.y
            while not rospy.is_shutdown():
                if state_ == 0:
                    fix_yaw(desired_position)
                    print(i)
                elif state_ == 1:
                    go_straight_ahead(desired_position)
                    print(i)
                elif state_ == 2:
                    print(i)
                    if i!=2:
                        done(0)
                        change_state(0)
                    else:
                        done(.2)
                        change_state(0)
                        active_=False

                    break
                else:
                    rospy.logerr('Unknown state!')
            rospy.loginfo("loop exit successful")
            rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    main()