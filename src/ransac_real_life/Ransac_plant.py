#!/usr/bin/env python
import rospy
import time
import math
from tf import transformations
import tf
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
import matplotlib
matplotlib.use('GTK3Agg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool,SetBoolRequest,SetBoolResponse
from nav_msgs.msg import Odometry
laser_ranges = []
laser_angles = []

# bp= [[-2,1],[2,3],[-2,-3],[2,-1],[2.5,-3],[3.5,3]]
# bp= [[0,0.5],[3.5,2.5],[0,-2.5],[3.5,-0.5],[2.5,-2.5],[3.5,2.5]]
# bp= [[0.0,0.0],[4,1],[0.0,-1],[4,0.0]] #,[0.5,-0.33],[3.5,0.33]]
bp=[[-0.5,0.15],[2.5,0.85],[-0.5,-0.85],[2.5,-0.15]] 
# bp=[[0.0,0.15],[2.5,0.85],[0.0,-0.85],[2.5,-0.15]] 
# bp=[[-3,-1],[-1,3],[1,-1],[3,3]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2= bp[3][0]-bp[2][0]
height2= bp[3][1]-bp[2][1]
# width3= bp[5][0]-bp[4][0]
# height3=bp[5][1]-bp[4][1]
position_ = Point()
yaw_ = 0
state_ = 0
num_of_slope_= 1
right_dist_avg = 0
left_dist_avg = 0

# rectangle = Rectangle(bp[0], width, height, linewidth=1, edgecolor='r', facecolor='none')



def fit_with_least_squares(X, y):

    b = np.ones((X.shape[0], 1))
    A = np.hstack((X, b))
    theta = np.linalg.lstsq(A, y , rcond=None)[0]
    return theta

def evaluate_model(X, y, theta, inlier_threshold):

    b = np.ones((X.shape[0], 1))
    y = y.reshape((y.shape[0], 1))
    A = np.hstack((y, X, b))
    theta = np.insert(theta, 0, -1.)
    
    distances = np.abs(np.sum(A*theta, axis=1)) / np.sqrt(np.sum(np.power(theta[:-1], 2)))
    inliers = distances <= inlier_threshold
    num_inliers = np.count_nonzero(inliers == True)
    
    return num_inliers
    
def ransac(X, y, max_iters=50, samples_to_fit=2, inlier_threshold=0.17, min_inliers=20):  

    best_model = None
    best_model_performance = 0
    
    num_samples = X.shape[0]
    # print("number of samples: ",num_samples)
    
    for i in range(max_iters):
        sample = np.random.choice(num_samples, size=samples_to_fit, replace=False)
        if len(X[sample]) == 0 or len(y[sample]) == 0:
            break
        model_params = fit_with_least_squares(X[sample], y[sample])
        model_performance = evaluate_model(X, y, model_params, inlier_threshold)
        
        if model_performance < min_inliers:
            continue
        
        if model_performance > best_model_performance:
            best_model = model_params
            best_model_performance = model_performance
    
    return best_model


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
def range_filter(msg):
    global right_dist_avg,left_dist_avg
    ranges=msg.ranges
    ranges = [x for x in ranges if not math.isnan(x)]
    custom_range = ranges[260:360]+ranges[0:100]
    right_dist = [x for x in custom_range[0:70] if x<.85]
    left_dist=[x for x in custom_range[130:200] if x<.85]
    if len(right_dist)==0 and len(left_dist)==0:
        print('no data points for distance found')
        right_dist_avg=0
        left_dist_avg=0
    elif len(right_dist)==0 and len(left_dist)>0: 
        right_dist_avg = 0
        left_dist_avg = sum(left_dist)/len(left_dist)
        # print('Left :',left_dist_avg)
    elif len(right_dist)>0 and len(left_dist)==0:
        right_dist_avg = sum(right_dist)/len(right_dist)
        left_dist_avg = 0
        # print('Right :',right_dist_avg)
    elif len(right_dist)>0 and len(left_dist)>0:
        right_dist_avg = sum(right_dist)/len(right_dist)
        left_dist_avg = sum(left_dist)/len(left_dist)
    

def scan_callback(msg):

    global laser_ranges
    global laser_angles
    global X
    global y
    global bp
    global state_
    start_time = rospy.Time.now()
    laser_ranges = msg.ranges 
    global num_of_slope_
    global position_
    # laser_ranges = laser_ranges[270:360]+laser_ranges[0:90]

    


    # laser_ranges = [30 if math.isnan(x) or math.isinf(x) else x for x in laser_ranges ] 

    lines_m = []  # list for storing slope (m) values of all lines
    lines_b = []  # list for storing y-intercept (b) values of all lines
    X_all = [] 
    y_all = []
    
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
        # print(y.shape[0])

        if (X.shape[0]<=2) or (y.shape[0]<=2):
            continue
        else:

            result = ransac(X, y)
            # print('result is: ',result)
            if result is None:
                continue
            X_all.append(X)
            y_all.append(y)
            # print('result = ',result)
            m = result[0][0]
            # print('slope: ',m)
            b = result[1][0]
            # print('y axis part: ', b)
            lines_m.append(m)  # store slope value for current line
            
            lines_b.append(b)  # store y-intercept value for current line

    # print('length of m = ',len(lines_m))   
    plt.clf()
    plt.ylim([-2, 2])
    plt.xlim([-1, 3])
    plt.grid()
    plt.title("Ransac lines on the plot!")
    # plt.show()
    rectangle1 = Rectangle(bp[0], width1, height1, linewidth=1, edgecolor='r', facecolor='none')
    rectangle2 = Rectangle(bp[2], width2, height2, linewidth=1, edgecolor='b', facecolor='none')
    # rectangle3 = Rectangle(bp[4], width3, height3, linewidth=1, edgecolor='g', facecolor='none')
    ax = plt.gca()
    ax.add_patch(rectangle1)
    ax.add_patch(rectangle2)
    # ax.add_patch(rectangle3)
    # plt.plot(X, y,'bx')
    for i in range(len(X_all)):
        plt.scatter(X_all[i], y_all[i])
    # plot all the lines found by the loop using the stored slope and y-intercept values
        for m, b in zip(lines_m, lines_b):
            plt.plot(X_all[i], m*X_all[i]+b,'r', linewidth=5)
    p = 1
    num_of_slope_=len(lines_m)
    if len(lines_m)==2:
        m1 = lines_m[0] # slope of 1st line
        # print('slope 1 is: ',m1)
        b1 = lines_b[0] # y intersection of 1st line
        # print('b1 : ',b1)
        y1 = p * m1 + b1
        # print("y1 : ",y1)
        point1 = [p,y1]
        m2 = lines_m[1] # slope of 2nd line 
        # print('slope 2 is: ',m2)
        b2 = lines_b[1] # y intersection of 2nd line
        # print('b2 : ',b2)
        y2 = p * m2 + b2
        # print("y2 : ",y2)
        point2 = [p,y2]
        mid_point_y = (y1+y2)/2
        mid_point_base = [p,mid_point_y]
        # print("mid point in base footprint: x = {}, y= {}".format(mid_point_base[0],mid_point_base[1]))

     
    elif len(lines_m)==1:
        current_dist_1=math.fabs(lines_b[0]/(math.sqrt((lines_m[0]**2)+1)))
        # print("current distance from line 1:  ",current_dist_1)
        m1 = lines_m[0] # slope of 2nd line 
        # print('slope 1 is: ',m1)
        b1 = lines_b[0] # y intersection of 2nd line
        # print('b1 : ',b1)
        if current_dist_1 >0.3:
            mid_point_y = m1*p
            mid_point_base = [p,mid_point_y]
            # print("mid point in base footprint: x = {}, y= {}".format(mid_point_base[0],mid_point_base[1]))
        else:
            mid_point_y = m1*p+(-b1)

    elif len(lines_m)==0 and .5<position_.x<9.5:
        if right_dist_avg <0.3 or left_dist_avg >0.5:
            mid_point_y = 0.2
            p = .75
        elif right_dist_avg>0.5 or left_dist_avg<0.3:
            mid_point_y= -0.2
            p=0.75
    
    # mid_point_y = (y1+y2)/2
    # mid_point_base = [p,mid_point_y] # mid_point at x=3 
    # print("mid point in base footprint: x = {}, y= {}".format(mid_point_base[0],mid_point_base[1]))
    if len(lines_m)>0:
        # print('length of m: ',len(lines_m))

        plt.scatter(p,mid_point_y)
        plt.draw()
        plt.pause(0.001) 
    

        # angle_1 =  math.atan(lines_m[0])*(180/math.pi)
        # print("angle in degrees", angle_1)
        listener = tf.TransformListener()
        point = PointStamped()
        point.header.frame_id = 'base_link'
        point.point.x = p
        point.point.y = mid_point_y
        point.point.z = 0.0
        listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(4.0))
        point_odom = listener.transformPoint('odom', point)
        rospy.loginfo("mid point in odom frame: x=%f, y=%f", point_odom.point.x, point_odom.point.y)

        #publishing the mid point in the odom frame as a goal to move towards
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=2)
        point_msg = Point()
        point_msg.x = point_odom.point.x
        point_msg.y = point_odom.point.y
        point_msg.z = 0.0

        pub.publish(point_msg)

    elif len(lines_m)==0 and .5<position_.x<9.5:
        plt.scatter(p,mid_point_y)
        plt.draw()
        plt.pause(0.001) 
    

        # angle_1 =  math.atan(lines_m[0])*(180/math.pi)
        # print("angle in degrees", angle_1)
        listener = tf.TransformListener()
        point = PointStamped()
        point.header.frame_id = 'base_link'
        point.point.x = p
        point.point.y = mid_point_y
        point.point.z = 0.0
        listener.waitForTransform('odom', 'base_link', rospy.Time(), rospy.Duration(4.0))
        point_odom = listener.transformPoint('odom', point)
        rospy.loginfo("mid point in odom frame: x=%f, y=%f", point_odom.point.x, point_odom.point.y)

        #publishing the mid point in the odom frame as a goal to move towards
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=2)
        point_msg = Point()
        point_msg.x = point_odom.point.x
        point_msg.y = point_odom.point.y
        point_msg.z = 0.0

        pub.publish(point_msg)




    


if __name__ == '__main__':
    position_ , num_of_slope_
    rospy.init_node('lidar_ransac_transform', anonymous=True)
    sub_laser_1 = rospy.Subscriber('/scan', LaserScan, scan_callback,queue_size=2)
    sub_laser_2 = rospy.Subscriber('/scan', LaserScan, range_filter,queue_size=2)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    pub = rospy.Publisher('slope_number', Int32, queue_size=10)
    rate = rospy.Rate(20)  # 10 Hz
    while not rospy.is_shutdown():
          # Replace with your integer value
        # print(position_.x)
        pub.publish(num_of_slope_)
        print('number of slope: ',num_of_slope_)
        print('position x: ',position_.x)
        print('position y: ',position_.y)
        print('yaw: ',yaw_*180/math.pi)
        rate.sleep()
    # rospy.spin()
    
 