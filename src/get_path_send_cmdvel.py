#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
from numpy.lib.function_base import percentile
import rospy
import math
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Point, PoseStamped, Quaternion
from visualization_msgs.msg import Marker,MarkerArray
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from dynamic_reconfigure.server import Server
# from ucar_nav.cfg import pdConfig
# from ucar_nav.msg import damn
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionGoal
import threading
class control:
    def __init__(self):
        self.current_pose = rospy.Subscriber(
            '/model_odom',
            Odometry,
            self.read_current_position,
            queue_size=1)
        self.path_pose = rospy.Subscriber('/ugv_planner_node/trajectory_for_tracking',
                                          Path,
                                          self.callback_read_path,
                                          queue_size=1)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.init_markers_goal()
        self.init_tf()
        self.path_info = []
        self.last_vel = 0
        self.last_ang = 0
        self.v_Kp = 1.5
        self.v_Kd = 0.2
        self.a_Kp = 1.8  # 1.8
        self.a_Kd = 0.2
        self.v_Kp2 = 1.4  #1.2
        self.v_Kd2 = 0.2
        self.a_Kp2 =3
        self.a_Kd2 = 0.2
        self.back_time=time.time()
        self.front_time=time.time() #前帧时间
        self.last_goalid=0
        self.get_path=0
    def init_tf(self):
        self.listener = tf.TransformListener()
        self.stamped_in = PoseStamped()
        self.stamped_out = PoseStamped()
        self.stamped_in.header.frame_id = "/world"

    def init_markers_goal(self):
        marker_scale = 0.1
        marker_lifetime =1  # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 238.0, 'g': 118, 'b': 0.0, 'a': 1.0}
        self.marker_pub = rospy.Publisher('current_goal',
                                          Marker,
                                          queue_size=1)
        self.marker = Marker()
        self.marker.ns = marker_ns
        self.marker.id = marker_id
        self.marker.type = Marker.ARROW
        self.marker.action = Marker.ADD
        self.marker.lifetime = rospy.Duration(marker_lifetime)
        self.marker.scale.x = marker_scale
        self.marker.scale.y = marker_scale
        self.marker.color.r = 0
        self.marker.color.g = 1
        self.marker.color.b = 0
        self.marker.color.a = 1
        self.marker.header.frame_id = 'world'
        self.marker.header.stamp = rospy.Time.now()

    def dist(self, p1, p2):
        try:
            return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        except:
            return 0.5

    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)  # abcos
        sin_ang = np.linalg.norm(np.cross(v1, v2))  # absin
        return np.arctan2(sin_ang, cos_ang)  # 向量叉乘除以向量点乘得到两个向量之间的角度


    def callback_read_path(self, data):
        self.path_info = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_z = path_pose.pose.position.z-0.115#z是头顶到最高的值要减去0.115
            path_foot =float(path_pose.header.seq)/(100.0)
            (r, p, path_yaw) = tf.transformations.euler_from_quaternion([
                                    path_pose.pose.orientation.x,
                                    path_pose.pose.orientation.y,  
                                    path_pose.pose.orientation.z, 
                                    path_pose.pose.orientation.w])   
            self.path_info.append([float(path_x), float(path_y),float(path_yaw),float(path_z),float(path_foot)])
        print("get_path")
        self.get_path=1
    def read_current_position(self, data):
        if(self.get_path==1):
            x = data.pose.pose.position.x  # x是当前位置
            y = data.pose.pose.position.y
            z = data.pose.pose.position.z
            (r, p, yaw) = tf.transformations.euler_from_quaternion([
                                    data.pose.pose.orientation.x,
                                    data.pose.pose.orientation.y,  
                                    data.pose.pose.orientation.z, 
                                    data.pose.pose.orientation.w])   

            # base_link=tf.transformations.quaternion_matrix(np.array(([
            #                         data.pose.pose.orientation.x,
            #                         data.pose.pose.orientation.y,  
            #                         data.pose.pose.orientation.z, 
            #                         data.pose.pose.orientation.w])))
            # print(r, p, yaw)
            # print(base_link)


            path_points_x = [float(point[0]) for point in self.path_info]  # 一条路径上所有点的x，
            path_points_y = [float(point[1]) for point in self.path_info]
            path_points_yaw = [float(point[2]) for point in self.path_info]
            path_points_z = [float(point[3]) for point in self.path_info]
            path_points_foot = [float(point[4]) for point in self.path_info]

            LOOKAHEAD_DISTANCE = 0.5
            dist_array = np.zeros(len(self.path_info))
            distarray_minus_lookahead_dis = np.ones(len(self.path_info))*100000
            
            # for i in range(self.last_goalid,len(self.path_info),1):#避免选到之前的走过的goal，所以从上一次选过的开始选
            #     dist_array[i] = self.dist((path_points_x[i], path_points_y[i]),(x,y))
            #     distarray_minus_lookahead_dis[i] = abs(dist_array[i] -LOOKAHEAD_DISTANCE)
            end=0
            if(self.last_goalid+20>len(self.path_info)):
                end=len(self.path_info)
            else:
                end=self.last_goalid+20
            #用self.last_goalid+20 是因为上扶梯的时候会整不好
            for i in range(self.last_goalid,end,1):#避免选到之前的走过的goal，所以从上一次选过的开始选
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]),(x,y))
                distarray_minus_lookahead_dis[i] = abs(dist_array[i] -LOOKAHEAD_DISTANCE)

            goalid = np.argmin(distarray_minus_lookahead_dis)
            self.marker.pose.position.x      = path_points_x[goalid]
            self.marker.pose.position.y      = path_points_y[goalid]
            self.marker.pose.position.z      = path_points_z[goalid];
            quat= tf.transformations.quaternion_from_euler(0,0,path_points_yaw[goalid])
            self.marker.pose.orientation.x=  quat[0]
            self.marker.pose.orientation.y=  quat[1]
            self.marker.pose.orientation.z=  quat[2]
            self.marker.pose.orientation.w=  quat[3]
            self.marker_pub.publish(self.marker)


            #得把current_goal从世界坐标系转到baselink下，再做np.arctan2(delta_y, delta_x)
            print("goalid ",goalid)
            dist_proper = dist_array[goalid]
            delta_x = path_points_x[goalid]-x
            delta_y = path_points_y[goalid]-y
            delta_yaw=path_points_yaw[goalid]-yaw
            a=delta_x
            b=delta_y
            d=a*math.cos(yaw)+b*math.sin(yaw)#new y
            c=a*math.sin(yaw)-b*math.cos(yaw)#new x
            # print("c  ",c)
            # print("d  ",d)
            # print("                            arctan2     ",180*((np.arctan2(d, c)-1.57)/3.14))
            temp=180*((np.arctan2(d, c)-1.57)/3.14)
            if(180*((np.arctan2(d, c)-1.57)/3.14)<-180):
                print("+360")
                temp+=360
            # print("delta_yaw  ",delta_yaw)
            # print("                            arctan2     ",np.arctan2(delta_y, delta_x))
            vel_d = data.twist.twist.linear.x - self.last_vel
            ang_d = data.twist.twist.angular.z - self.last_ang

            move_cmd = Twist()
            move_cmd.linear.x = dist_proper * self.v_Kp + self.v_Kd * vel_d
            move_cmd.linear.z = path_points_z[goalid]#直接给位置好了，我记得大波罗实物高度就是给的位置而不是速度
            move_cmd.linear.y = path_points_foot[goalid]#借用一下消息格式
            move_cmd.angular.z = ((temp/180)*3.14) * self.a_Kp + self.a_Kd * ang_d
            self.last_vel = data.twist.twist.linear.x
            self.last_ang = data.twist.twist.angular.z
            self.last_goalid = goalid
            self.cmd_vel.publish(move_cmd)

if __name__ == "__main__":
    rospy.init_node("Controller")
    control()
    rospy.spin()
