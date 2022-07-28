/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "plan_manage/sdf_map.h"
#include "plan_manage/backward.hpp"
#include "plan_manage/State.h"
using namespace std;
using namespace Eigen;

namespace backward {
backward::SignalHandling sh;
}

class jump_library
{

    public:

        jump_library() = default;
        jump_library(const double h,double v,double r)//resolution
            : h_max(h),
              v_max(v),
              r_for_safe(r) {}

        TrajectoryStatePtr ** TraLibrary;//只需要两维，离散的高度是直接写出来的，径向的速度是输入的（匀速的）
        int v_discretize_step;
        int h_discretize_step;
        ros::Publisher _path_vis_pub;
        ros::Subscriber head_odom_sub,trigger_sub;
        double h_max,v_max,r_for_safe;
        nav_msgs::Odometry head_odometry;
        typedef std::shared_ptr<jump_library> Ptr;
        SDFMap::Ptr sdfptr_for_check_collision;
    private:

    public:
        void init(ros::NodeHandle& nh, SDFMap::Ptr &SDF_ptr) {
            _path_vis_pub  = nh.advertise<visualization_msgs::MarkerArray>("jump_library_vis",1); 
            head_odom_sub  = nh.subscribe("/model_odom",1 ,&jump_library::head_odomRcvCallback, this);
            trigger_sub    = nh.subscribe("/goal",1,&jump_library::trigger_Callback, this);
            sdfptr_for_check_collision = SDF_ptr;
        }

        void head_odomRcvCallback(const nav_msgs::Odometry odom)//从simulator那来的，暂时先注释掉一部分
        {
            head_odometry=odom;
            
        }
        void trigger_Callback(const geometry_msgs::PoseStamped initpt){
            trajectoryLibrary(h_max,v_max,head_odometry,r_for_safe);
        }   
    //z轴上我还是不用速度去离散了，不跳的时候z轴都是靠头伸缩搞出来的，跳跃之前的轨迹末状态z的速度没卵用
    //但是径向上的速度需要输入，跳跃之前的轨迹末状态的速度决定了能跳多远
        void trajectoryLibrary(const double h_max,double v_max,nav_msgs::Odometry head_odometry,double r_for_safe)
        {
            Vector3d start_pt;
            start_pt(0)=head_odometry.pose.pose.position.x;
            start_pt(1)=head_odometry.pose.pose.position.y;
            start_pt(2)=head_odometry.pose.pose.position.z;
            
            tf::Quaternion quat;
            tf::quaternionMsgToTF(head_odometry.pose.pose.orientation,quat);
            double roll,pitch,yaw;
            tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

            Vector3d acc_input;
            Vector3d pos;
            v_discretize_step=v_max/0.05;
            h_discretize_step=h_max/0.1;//比如最高点h从0到0.6 
            std::cout<<"v_discretize_step "<<v_discretize_step<<std::endl;
            std::cout<<"h_discretize_step "<<h_discretize_step<<std::endl;
            double _time_step=100;


            TraLibrary  = new TrajectoryStatePtr * [h_discretize_step + 1];     
            for(int i=0; i <=h_discretize_step; i++)
            { 
                TraLibrary[i] = new TrajectoryStatePtr  [v_discretize_step + 1];
                double h=i*0.05;
                double _time_interval=2*sqrt(2*h/9.8);  //最高点自由落体到水平0点的时间的2倍
                double v_2d_min=2*r_for_safe/_time_interval;//最少跳一个直径那么远
                std::cout<<"v_2d_min "<<v_2d_min<<std::endl;
                double v_h_start=sqrt(2*h*9.8);
                double delta_time;
                delta_time = (_time_interval / double(_time_step))*2;//加时一倍，这样让轨迹线在出发点下面也有
                double v_h_now;
                for(int j=0;j <= v_discretize_step; j++)
                    {   
                        vector<Vector3d> Position;
                        double v_2d=j*0.05;
                        bool collision = false;
                        if(v_2d>v_2d_min)
                        {
                            pos(0) = start_pt(0);
                            pos(1) = start_pt(1);
                            pos(2) = start_pt(2);
                            Position.push_back(pos);
                            double last_p0=pos(0);
                            double last_p1=pos(1);
                            double last_p2=pos(2);
                            for(int step=0 ; step<=_time_step ; step ++)
                            {
                            v_h_now=v_h_start-9.8*step*delta_time;
                            pos(0) = last_p0+v_2d*delta_time*cos(yaw);
                            pos(1) = last_p1+v_2d*delta_time*sin(yaw);
                            pos(2) = last_p2-(1/2)*9.8*delta_time*delta_time+v_h_now*delta_time;
                            last_p0=pos(0);
                            last_p1=pos(1);
                            last_p2=pos(2);
                            Position.push_back(pos);
                            //check球内是否无碰撞，即check球心的esdf最小障碍物距离是否大于r 
                            //这条路上有一处撞了那就撞了
                            //而且不能一直check，只check起跳到落地期间的，但起跳的时候可能是蹲着的..会被判成occ..
                            double dist=sdfptr_for_check_collision->getDistance(pos);
                            if(dist<r_for_safe){
                                collision = true;
                            }
                            }                      
                        }



                        TraLibrary[i][j] = new TrajectoryState(Position);
                        
                        //if there is not any obstacle in the trajectory we need to set 'collision_check = true', so this trajectory is useable
                        if(collision)
                            TraLibrary[i][j]->setCollision();
                        
                        //record the min_cost in the trajectory Library, and this is the part pf selecting the best trajectory cloest to the planning traget
                        // if(TraLibrary[i][j]->collision_check == false){

                        // }
                    }
            }
            
            visTraLibrary(TraLibrary);
            return;
        }


        void visTraLibrary(TrajectoryStatePtr ** TraLibrary)
        {
            double _resolution = 0.2;
            visualization_msgs::MarkerArray  LineArray;
            visualization_msgs::Marker       Line;

            Line.header.frame_id = "world";
            Line.header.stamp    = ros::Time::now();
            Line.ns              = "demo_node/TraLibrary";
            Line.action          = visualization_msgs::Marker::ADD;
            Line.pose.orientation.w = 1.0;
            Line.type            = visualization_msgs::Marker::LINE_STRIP;
            Line.scale.x         = _resolution/5;

            Line.color.r         = 0.0;
            Line.color.g         = 0.0;
            Line.color.b         = 1.0;
            Line.color.a         = 1.0;

            int marker_id = 0;

            for(int i = 0; i <= h_discretize_step; i++){
                // int j = v_discretize_step;
                for(int j = 0; j<= v_discretize_step;j++){  
                    
                        if(TraLibrary[i][j]->collision_check == false){
                                Line.color.r         = 0.0;
                                Line.color.g         = 0.0;
                                Line.color.b         = 1.0;
                                Line.color.a         = 1.0;
 
                        }else{
                            Line.color.r         = 1.0;
                            Line.color.g         = 0.0;
                            Line.color.b         = 0.0;
                            Line.color.a         = 1.0;
                        }
                        Line.points.clear();
                            geometry_msgs::Point pt;
                            Line.id = marker_id;
                            for(int index = 0; index < int(TraLibrary[i][j]->Position.size());index++){
                                Vector3d coord = TraLibrary[i][j]->Position[index];
                                pt.x = coord(0);
                                pt.y = coord(1);
                                pt.z = coord(2);
                                Line.points.push_back(pt);
                            }
                            LineArray.markers.push_back(Line);
                            _path_vis_pub.publish(LineArray);
                            ++marker_id; 
                    
                }
            }    
        }








     
   

};


