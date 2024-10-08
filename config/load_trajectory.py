#!/usr/bin/python3

import argparse
import rospy
import rosnode
import random
import os
import math

from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.msg import Reference

selector = "line" 
# selector = "eight"
# selector = "star"

# Instantiate the parser
parser = argparse.ArgumentParser(description='Optional app description')

parser.add_argument('--loop', action='store_true', default=False)
parser.add_argument('--fly_now', action='store_true', default=False)
parser.add_argument('-s', '--selector')
parser.add_argument('-e', '--exec_time', type=float, help='a')

args = parser.parse_args()
# 

class Goto:

    def __init__(self):

        rospy.init_node('goto', anonymous=True)

        rospy.loginfo('ros not initialized')

        publishers = []
        n_uavs = 1

        ## | --------------------- service clients -------------------- |

        # self.sc_path = rospy.ServiceProxy('/uav42/trajectory_generation/path', PathSrv)
        # self.sc_path = rospy.ServiceProxy('/uav1/trajectory_generation/path', PathSrv)
        self.sc_path = rospy.ServiceProxy('/uav2/trajectory_generation/path', PathSrv)

        path_msg = PathSrvRequest()

        path_msg.path.header.frame_id = ""
        path_msg.path.header.stamp = rospy.Time.now()
        # print(args.fly_now)
        path_msg.path.use_heading = True
        path_msg.path.fly_now = args.fly_now
        # path_msg.path.fly_now = True

        path_msg.path.max_execution_time = args.exec_time
#        path_msg.path.max_execution_time = 50.0
        path_msg.path.max_deviation_from_path = 0.1
        path_msg.path.dont_prepend_current_state = True 
        # path_msg.path.loop =True
        path_msg.path.loop = args.loop


        if args.selector == "line":
            center_point = [4.0, 0.0, 1.0, 0.0]
            dist = 15.0
            point1 = Reference()
            point2 = Reference()
            along_x = False
            
            point1.position.x = center_point[0]
            point1.position.y = center_point[1]
            point1.position.z = center_point[2] 
            point1.heading = 0
            point2.position.x = center_point[0]
            point2.position.y = center_point[1]
            point2.position.z = center_point[2] 
            point2.heading = 0
            if along_x:
                point1.position.x = center_point[0] - dist/2
                point2.position.x = center_point[0] + dist/2
            else: 
                point1.position.y = center_point[0] - dist/2
                point2.position.y = center_point[0] + dist/2

            path_msg.path.points.append(point1)
            path_msg.path.points.append(point2)
            path_msg.path.points.append(point1)

        elif args.selector == "eight":
            center_point = [4.0, 0.0, 4,0, 5.0]
            radius = 2.5
            along_x = False

            angle = -math.pi / 2.0
            data_points = 6
            angle_discretization =(2*math.pi) / float(data_points)  
            first_pt = Reference()
            if along_x:
                for i in range(data_points):
                    point_circle = Reference()
                    point_circle.position.x = center_point[0] + radius * math.sin(angle) + radius
                    point_circle.position.y = center_point[1]
                    point_circle.position.z = center_point[2] + radius * math.cos(angle)
                    point_circle.heading = center_point[3]
                    angle = angle + angle_discretization
                    path_msg.path.points.append(point_circle)
                    if i == 0:
                        first_pt = Reference()
                        first_pt.position.x = point_circle.position.x
                        first_pt.position.y = point_circle.position.y
                        first_pt.position.z = point_circle.position.z
                        first_pt.heading = point_circle.heading
                angle_discretization =(2*math.pi) / float(data_points) * -1.0
                angle = math.pi / 2.0
                for i in range(data_points):
                    point_circle = Reference()
                    point_circle.position.x = center_point[0] + radius * math.sin(angle) - radius
                    point_circle.position.y = center_point[1]
                    point_circle.position.z = center_point[2] + radius * math.cos(angle)
                    point_circle.heading = center_point[3]
                    path_msg.path.points.append(point_circle)
                    angle = angle + angle_discretization
            else:
                for i in range(data_points):
                    point_circle = Reference()
                    point_circle.position.x = center_point[0]
                    point_circle.position.y = center_point[1] + radius * math.sin(angle) + radius
                    point_circle.position.z = center_point[2] + radius * math.cos(angle)
                    point_circle.heading = center_point[3]
                    angle = angle + angle_discretization
                    path_msg.path.points.append(point_circle)
                    if i == 0:
                        first_pt = Reference()
                        first_pt.position.x = point_circle.position.x
                        first_pt.position.y = point_circle.position.y
                        first_pt.position.z = point_circle.position.z
                        first_pt.heading = point_circle.heading
                angle_discretization =(2*math.pi) / float(data_points) * -1.0
                angle = math.pi / 2.0
                for i in range(data_points):
                    point_circle = Reference()
                    point_circle.position.x = center_point[0]
                    point_circle.position.y = center_point[1] + radius * math.sin(angle) - radius
                    point_circle.position.z = center_point[2] + radius * math.cos(angle)
                    point_circle.heading = center_point[3]
                    angle = angle + angle_discretization
                    path_msg.path.points.append(point_circle)
                
            path_msg.path.points.append(first_pt)

        elif args.selector == "star":
            rospy.loginfo('TBD')
            
        
        else:
            rospy.loginfo('no trajectory selected')

        try:
            response = self.sc_path.call(path_msg)
        except:
            rospy.logerr('[SweepingGenerator]: path service not callable')
            pass

if __name__ == '__main__':
    try:
        goto = Goto()
    except rospy.ROSInterruptException:
        pass
