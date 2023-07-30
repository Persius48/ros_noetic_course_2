#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import time
import math
import matplotlib
matplotlib.use('GTK3Agg')
import tf
from tf import transformations
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from sensor_msgs.msg import LaserScan


laser_ranges = []
laser_angles = []

# bp = [[-0.1, 1.9], [1.3, 2.2], [1.8, 1.8], [3.1, 2.1], [1.75, 1.9], [1.95, 3.0]]
# bp = [[-5,-3],[5,3]]
# bp = [[-.7,0],[.7,3]]
# bp= [[-2,1],[2,3],[-2,-3],[2,-1],[2.5,-3],[3.5,3]]
# bp= [[0,0.5],[3.5,2.5],[0,-2.5],[3.5,-0.5],[2.5,-2.5],[3.5,2.5]]
bp= [[0.0,0.15],[2.5,0.85],[0.0,-0.85],[2.5,-0.15]] #,[0.5,-0.33],[3.5,0.33]]
# bp=[[-3,-1],[-1,3],[1,-1],[3,3]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2= bp[3][0]-bp[2][0]
height2= bp[3][1]-bp[2][1]
# width3= bp[5][0]-bp[4][0]
# height3=bp[5][1]-bp[4][1]


# robot state variables
position_ = Point()
yaw_ = 0
counter1 = 0
counter2 = 0
# rectangle = Rectangle(bp[0], width, height, linewidth=1, edgecolor='r', facecolor='none')
def clbk_odom(msg):
    global position_
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    # print('current position_ x =',position_.x)
    # print('current position_ y =',position_.y)
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

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
    
def ransac(X, y, max_iters=50, samples_to_fit=2, inlier_threshold=0.23, min_inliers=6):  

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
    # return model_params
def turning_left(i):
    global counter1
    while position_.x < 10.8 and position_.x >9 and position_.y > Y1[i]-0.25 and position_.y < Y1[i]+0.25:
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = P1[0]
        point_msg.y = Y1[i]
        point_msg.z = 0.0
        pub.publish(point_msg)
    while position_.x > 10.8 and position_.y < Y1[i+1]-0.25 and position_.y > Y1[i]-0.25:
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = P1[0]
        point_msg.y = Y1[i+1]
        point_msg.z = 0.0
        pub.publish(point_msg)
    while position_.x <11.2 and position_.x>10.3 and position_.y > Y1[i+1]-0.25 and position_.y < Y1[i+1]+0.25:
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = P1[1]
        point_msg.y = Y1[i+1]
        point_msg.z = 0.0
        pub.publish(point_msg)
    counter1 +=1
    # print("counter after if/else =",counter)

def turning_right(i,k=0):
    global counter2
    while position_.x < 0.5 and position_.x >-0.8 and position_.y > Y2[i+k+1]-0.15 and position_.y < Y2[i+k+1]+0.15:
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = P2[0]
        point_msg.y = Y2[i+k+1]
        point_msg.z = 0.0
        pub.publish(point_msg)
    while position_.x < -0.8 and position_.y < Y2[i+k+2]-0.15 and position_.y > Y2[i+k+1]-0.15:
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = P2[0]
        point_msg.y = Y2[i-k+2]
        point_msg.z = 0.0
        pub.publish(point_msg)
    while position_.x <-0.2 and position_.x>-1.2 and position_.y > Y2[i-k+2]-0.15 and position_.y < Y2[i-k+2]+0.15:
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = P2[1]
        point_msg.y = Y2[i-k+2]
        point_msg.z = 0.0
        pub.publish(point_msg)
    counter2 += 1

def scan_callback(msg):

    global laser_ranges
    global laser_angles
    global X
    global y
    global bp
    global Y1
    global P1
    global P2
    global Y2
    global counter1
    global counter2
    start_time = rospy.Time.now()
    laser_ranges = msg.ranges 
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
            print('result = ',result)
            m = result[0][0]
            # print('slope: ',m)
            b = result[1][0]
            # print('y axis part: ', b)
            lines_m.append(m)  # store slope value for current line
            
            lines_b.append(b)  # store y-intercept value for current line

    print('length of m = ',len(lines_m))   
    plt.clf()
    plt.ylim([-4,4])
    plt.xlim([-0.5, 3])
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
    # P1 = [11,10.1]
    # start_point_y = 0.5
    # interval = 0.75
    # num_points = 10
    # Y1 = []
    # for i in range(num_points):
    #     point = start_point_y + i * interval
    #     Y1.append(point)
    # print(Y1)
    
    if len(lines_m)==2:   
        m1 = lines_m[0] # slope of 1st line
        # print('slope 1 is: ',m1)
        b1 = lines_b[0] # y intersection of 1st line
        # print('b1 : ',b1)
        y1 = p * m1 + b1
        print("y1 : ",y1)
        point1 = [p,y1]
        m2 = lines_m[1] # slope of 2nd line 
        # print('slope 2 is: ',m2)
        b2 = lines_b[1] # y intersection of 2nd line
        # print('b2 : ',b2)
        y2 = p * m2 + b2
        print("y2 : ",y2)
        point2 = [p,y2]
        mid_point_y = (y1+y2)/2
        print('mid_point_y', mid_point_y)
        mid_point_base = [p,mid_point_y]
        print("mid point in base footprint: x = {}, y= {}".format(mid_point_base[0],mid_point_base[1]))

        listener = tf.TransformListener()
        point = PointStamped()
        point.header.frame_id = "base_footprint"
        point.point.x = p
        point.point.y = mid_point_y
        point.point.z = 0.0
        listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(4.0))
        point_odom = listener.transformPoint('odom', point)
        rospy.loginfo("mid point in odom frame: x=%f, y=%f", point_odom.point.x, point_odom.point.y)

        #publishing the mid point in the odom frame as a goal to move towards
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = point_odom.point.x
        point_msg.y = point_odom.point.y
        point_msg.z = 0.0

        pub.publish(point_msg)

    
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
            print("mid point in base footprint: x = {}, y= {}".format(mid_point_base[0],mid_point_base[1]))
        else:
            mid_point_y = m1*p+(-b1)

        listener = tf.TransformListener()
        point = PointStamped()
        point.header.frame_id = "base_footprint"
        point.point.x = p
        point.point.y = mid_point_y
        point.point.z = 0.0
        listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(4.0))
        point_odom = listener.transformPoint('odom', point)
        rospy.loginfo("mid point in odom frame: x=%f, y=%f", point_odom.point.x, point_odom.point.y)

        #publishing the mid point in the odom frame as a goal to move towards
        pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        point_msg = Point()
        point_msg.x = point_odom.point.x
        point_msg.y = point_odom.point.y
        point_msg.z = 0.0

        pub.publish(point_msg)
    
    # elif counter ==0 and len(lines_m) == 0 and position_.x > 9:
    #     turning_left(0)
    #     counter += 1
    # elif counter ==1 and len(lines_m) == 0 and position_.x > 9:
    #     turning_left(2)
    #     counter += 1
    elif len(lines_m)==0 and position_.x > 9:
        # counter = 0
        P1 = [11,10.1]
        start_point_y = 0.4
        interval = 0.75
        num_points = 10
        Y1 = []
        for i in range(num_points):
            point = start_point_y + i * interval
            Y1.append(point)
        print(Y1)
        print("counter before if/else =",counter1)
        if counter1 == 0 and position_.y > Y1[0]-0.2 and position_.y < Y1[0]+0.2:
            turning_left(0)
            print('point1')
            
        elif counter1 == 1 and position_.y > Y1[2]-0.2 and position_.y < Y1[2]+0.2:
            turning_left(2)
            print('point2')
           
        elif counter1 == 2 and position_.y > Y1[4]-0.2 and position_.y < Y1[4]+0.2:
            turning_left(4)
            print('point3')
           
        elif counter1 == 3 and position_.y > Y1[6]-0.2 and position_.y < Y1[6]+0.2:
            turning_left(6)
            
         
        


        # if position_.y > 0 and position_.y < 1.5:
        #     turning_left(0)
        # elif position_.y > 1.5 and position_.y < 3:
        #     turning_left(2)
        # elif position_.y > 3 and position_.y < 4.5:
        #     turning_left(4)
        # while condition1:
        #     if position_.y > 4.5 and position_.y < 5.5:
        #         turning_left(6)
        #         condition1 = False
        #         condition2 = True
        # while condition2:
        #     if position_.y < 6.25:
        #         turning_left(7)
        #         condition2 = False


        # while position_.x < 10.8 and position_.x >9.5 and position_.y > Y1[0]-0.25 and position_.y < Y1[0]+0.25:
        #     pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        #     point_msg = Point()
        #     point_msg.x = P[0]
        #     point_msg.y = Y1[0]
        #     point_msg.z = 0.0
        #     pub.publish(point_msg)
        # while position_.x > 10.8 and position_.y < Y1[1]-0.25 and position_.y > Y1[0]-0.25:
        #     pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        #     point_msg = Point()
        #     point_msg.x = P[0]
        #     point_msg.y = Y1[1]
        #     point_msg.z = 0.0
        #     pub.publish(point_msg)
        # while position_.x <11.2 and position_.x>10.3 and position_.y > Y1[1]-0.25 and position_.y < Y1[1]+0.25:
        #     pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        #     point_msg = Point()
        #     point_msg.x = P[1]
        #     point_msg.y = Y1[1]
        #     point_msg.z = 0.0
        #     pub.publish(point_msg)

    elif len(lines_m)==0 and position_.x < 0.5:
        P2 = [-1,-0.1]
        start_point_y = 0
        interval = 0.75
        num_points = 10
        Y2 = []
        for i in range(num_points):
            point = start_point_y + i * interval
            Y2.append(point)
        print(Y2)
        
        if counter2 == 0 and position_.y > 0 and position_.y < 1.75: 
            turning_right(0)
        elif counter2 == 1 and position_.y > 1.75 and position_.y < 3.25:
            turning_right(2)
        elif counter2 == 2 and position_.y > 3.25 and position_.y < 4.75:
            turning_right(4)
        elif counter2 == 3 and position_.y > 4.75 and position_.y < 6.5:
            turning_right(6,1)
        elif counter2 == 4 and position_.y > 6.5:
            pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
            point_msg = Point()
            point_msg.x = P2[0]
            point_msg.y = Y2[10]
            point_msg.z = 0.0
            pub.publish(point_msg)
            print('Task_1 complete.')
        


        # while position_.x < 0.5 and position_.x >-0.8 and position_.y > Y2[1]-0.15 and position_.y < Y2[1]+0.15:
        #     pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        #     point_msg = Point()
        #     point_msg.x = P2[0]
        #     point_msg.y = Y2[1]
        #     point_msg.z = 0.0
        #     pub.publish(point_msg)
        # while position_.x < -0.8 and position_.y < Y2[2]-0.15 and position_.y > Y2[1]-0.15:
        #     pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        #     point_msg = Point()
        #     point_msg.x = P2[0]
        #     point_msg.y = Y2[2]
        #     point_msg.z = 0.0
        #     pub.publish(point_msg)
        # while position_.x <-0.2 and position_.x>-1.2 and position_.y > Y2[2]-0.15 and position_.y < Y2[2]+0.15:
        #     pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        #     point_msg = Point()
        #     point_msg.x = P2[1]
        #     point_msg.y = Y2[2]
        #     point_msg.z = 0.0
        #     pub.publish(point_msg)
        

        # dist1_pub = rospy.Publisher('current_dist_1', Float32, queue_size=1)
        # dist1_msg = Float32()
        # dist1_msg.data = current_dist_1
        # dist1_pub.publish(dist1_msg)
        # rospy.sleep(0.1)

    
    # mid_point_y = (y1+y2)/2
    # mid_point_base = [p,mid_point_y] # mid_point at x=3 
    # print("mid point in base footprint: x = {}, y= {}".format(mid_point_base[0],mid_point_base[1]))
    if len(lines_m)>0:
        plt.scatter(p,mid_point_y)
        plt.draw()
        plt.pause(0.001) 
    

        # angle_1 =  math.atan(lines_m[0])*(180/math.pi)
        # print("angle in degrees", angle_1)
        # listener = tf.TransformListener()
        # point = PointStamped()
        # point.header.frame_id = "base_footprint"
        # point.point.x = p
        # point.point.y = mid_point_y
        # point.point.z = 0.0
        # listener.waitForTransform('odom', 'base_footprint', rospy.Time(), rospy.Duration(4.0))
        # point_odom = listener.transformPoint('odom', point)
        # rospy.loginfo("mid point in odom frame: x=%f, y=%f", point_odom.point.x, point_odom.point.y)

        # #publishing the mid point in the odom frame as a goal to move towards
        # pub = rospy.Publisher('mid_point_goal', Point, queue_size=1)
        # point_msg = Point()
        # point_msg.x = point_odom.point.x
        # point_msg.y = point_odom.point.y
        # point_msg.z = 0.0

        # pub.publish(point_msg)


if __name__ == '__main__':
    rospy.init_node('lidar_ransac_transform', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    # msg = rospy.wait_for_message("/scan",LaserScan,timeout=None)
    # scan_callback(msg)
    rospy.spin()

