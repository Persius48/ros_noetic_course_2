#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
pub = None
laser_ranges = []
laser_angles = []
bp=[[0.5,0.125],[1.0,0.3125],[0.5,-0.3125],[1.0,-0.125]]
width1 = bp[1][0] - bp[0][0]
height1 = bp[1][1] - bp[0][1]
width2= bp[3][0]-bp[2][0]
height2= bp[3][1]-bp[2][1]

def clbk_laser(msg):
    global right_dist,right_dist_avg, left_dist,left_dist_avg,ranges
    row_width = 0.8
    ranges=msg.ranges
    ranges = [x for x in ranges if not math.isnan(x)]
    custom_range = ranges[270:360]+ranges[0:90]
    right_dist = [x for x in custom_range[0:70] if x<0.9]
    left_dist=[x for x in custom_range[110:180] if x<0.9]

    if len(right_dist)==0 and len(left_dist)==0:
        print('no data points for distance found')  
    elif len(right_dist)==0 and len(left_dist)>0: 
        left_dist_avg = sum(left_dist)/len(left_dist)
        right_dist_avg=0
        print('Right average :',right_dist_avg)
        print('Left Average :',left_dist_avg)
    elif len(right_dist)>0 and len(left_dist)==0:
        right_dist_avg = sum(right_dist)/len(right_dist)
        left_dist_avg = 0
        print('Right average :',right_dist_avg)
        print('Left Average :',left_dist_avg)
    elif len(right_dist)>0 and len(left_dist)>0:
        right_dist_avg = sum(right_dist)/len(right_dist)
        left_dist_avg = sum(left_dist)/len(left_dist)
        print('Right average :',right_dist_avg)
        print('Left Average :',left_dist_avg)
    regions = {
        'right':  min(min(custom_range[0:70]), 0.75),
        'right-1': min(min(custom_range[30:70]),0.75),
        # 'right-2':min(min(custom_range[0:30]),0.75),
        'fright': min(min(custom_range[70:85]), 1),
        'front':  min(min(custom_range[70:110]), 1),
        'forfront': min(min(custom_range[85:95]),1),
        'fleft':  min(min(custom_range[95:110]), 1),
        'left':   min(min(custom_range[110:180]), 0.75),
        'left-1': min(min(custom_range[110:150]),0.75),
        # 'left-2': min(min(custom_range[150:180]),0.75),
    }   

    print('regions are: ',regions)

def front_boxes(msg):
    global laser_ranges,laser_angles,X_right,y_right,X_left,y_left
    # X_right = [] 
    # y_right = []
    # X_left=[]
    # y_left=[]
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
        else:
            X_right=X
            y_right=y

    print('number of points detected in left box',X_left.shape[0])
    print('number of points detected in right box',X_right.shape[0])
    
    # print('length of m = ',len(lines_m))   
    plt.clf()
    plt.ylim([-0.8, 0.8])
    plt.xlim([-0, 1.5])
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

    plt.scatter(X_left, y_left)
    plt.scatter(X_right, y_right)
    plt.draw()
    plt.pause(0.001) 
    


def main():
    global pub
    
    rospy.init_node('reading_laser')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    sub = rospy.Subscriber('/scan', LaserScan, front_boxes)
    rospy.spin()

if __name__ == '__main__':
    main()