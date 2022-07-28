#include <ros/ros.h>
#include <thread>
#include <cstdlib>
#include <ctime>
#include "OsqpEigen/OsqpEigen.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <stdlib.h>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
// #include <pcl/search/kdtree.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/features/integral_image_normal.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
// #include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "plan_manage/sdf_map.h"
#include "std_msgs/Int8.h" //#include "std_msgs/String.h"
#include "plan_manage/sfc_gen.hpp"
#include "misc/visualizer.hpp"
#include "plan_manage/gcopter.hpp"
#include "plan_manage/jump_library.hpp"
#include "mpc/Polynome.h"
using namespace std;
using namespace Eigen;
        struct mapConfig
    {
        std::string subscribe_cloudTopic;
        std::string pub_surface_grid_Topic1,pub_surface_grid_Topic2;
        std::string pub_esdf_Topic;
        std::string pub_feasible_grid_Topic;
        std::string sub_goal;
        double resolution;
        std::vector<double> mapBound;
        int esdf_vis_slice_height_;
        double robot_height_min;
        double robot_height_max;
        double robot_radius;
        double robot_wheel_radius;
        double robot_jump_highest;
        double robot_jump_v2d_max;

        double timeoutRRT;
        double maxVelMag;
        double maxBdrMag;
        double maxTiltAngle;
        double minThrust;
        double maxThrust;
        double vehicleMass;
        double gravAcc;
        double horizDrag;
        double vertDrag;
        double parasDrag;
        double speedEps;
        double weightT;
        std::vector<double> chiVec;
        double smoothingEps;
        int integralIntervs;
        double relCostTol;

        mapConfig(const ros::NodeHandle &nh_priv)
        {
            nh_priv.getParam("subscribe_cloudTopic", subscribe_cloudTopic);
            nh_priv.getParam("pub_surface_grid_Topic1", pub_surface_grid_Topic1);
            nh_priv.getParam("pub_surface_grid_Topic2", pub_surface_grid_Topic2);
            nh_priv.getParam("pub_esdf_Topic", pub_esdf_Topic);
            nh_priv.getParam("pub_feasible_grid_Topic", pub_feasible_grid_Topic);
            nh_priv.getParam("resolution", resolution);
            nh_priv.getParam("mapBound", mapBound);
            nh_priv.getParam("esdf_vis_slice_height_", esdf_vis_slice_height_);
            nh_priv.getParam("sub_goal", sub_goal);
            nh_priv.getParam("robot_height_min", robot_height_min);
            nh_priv.getParam("robot_height_max", robot_height_max);
            nh_priv.getParam("robot_radius", robot_radius);
            nh_priv.getParam("robot_wheel_radius", robot_wheel_radius);
            nh_priv.getParam("robot_jump_highest", robot_jump_highest);
            nh_priv.getParam("robot_jump_v2d_max", robot_jump_v2d_max);



            nh_priv.getParam("TimeoutRRT", timeoutRRT);
            nh_priv.getParam("MaxVelMag", maxVelMag);
            nh_priv.getParam("MaxBdrMag", maxBdrMag);
            nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
            nh_priv.getParam("MinThrust", minThrust);
            nh_priv.getParam("MaxThrust", maxThrust);
            nh_priv.getParam("VehicleMass", vehicleMass);
            nh_priv.getParam("GravAcc", gravAcc);
            nh_priv.getParam("HorizDrag", horizDrag);
            nh_priv.getParam("VertDrag", vertDrag);
            nh_priv.getParam("ParasDrag", parasDrag);
            nh_priv.getParam("SpeedEps", speedEps);
            nh_priv.getParam("WeightT", weightT);
            nh_priv.getParam("ChiVec", chiVec);
            nh_priv.getParam("SmoothingEps", smoothingEps);
            nh_priv.getParam("IntegralIntervs", integralIntervs);
            nh_priv.getParam("RelCostTol", relCostTol);
        }
    };
  
  class UGVPlannerManager
  {
    private:
        ros::NodeHandle nh;
        ros::Subscriber mapSub,target_sub,odom_sub;
        ros::Publisher pub_surface_cloud1,pub_surface_cloud2,angle_color_cloud_pub_,astar_path_vis_pub;
        ros::Publisher path_For_tracking_pub,pub_cloud_what_you_Want,path_For_tracking_vis_pub;
        bool get_map=false;
        bool has_target=false;
        bool has_odom=false;
        bool init_all=false;
        mapConfig mconfig;
        Eigen::Vector3d now_pos;
        Eigen::Vector3d target_pos;
    public:
      double trajStamp;
      Trajectory<5> traj;
      vector<Eigen::Vector3d> end_Feasible_surf;
      std::vector<Eigen::Vector3d> end_feasible_set;
      std::vector<Eigen::Vector3d> end_feasible_edge_set;
      Visualizer visualizer;
      voxel_map::VoxelMap::Ptr voxelMapptr;
      SDFMap::Ptr sdf_ptr;
      jump_library::Ptr jump_lib_ptr;
      ros::Publisher traj_pub;
      UGVPlannerManager(const mapConfig &conf,  
                        ros::NodeHandle &nh_)
          : mconfig(conf),
            nh(nh_),
            visualizer(nh)
        {
            std::cout<<"new UGVPlannerManager"<<std::endl;

            const Eigen::Vector3i xyz((mconfig.mapBound[1] - mconfig.mapBound[0]) / mconfig.resolution,
                                      (mconfig.mapBound[3] - mconfig.mapBound[2]) / mconfig.resolution,
                                      (mconfig.mapBound[5] - mconfig.mapBound[4]) / mconfig.resolution);

            const Eigen::Vector3d offset(mconfig.mapBound[0], mconfig.mapBound[2], mconfig.mapBound[4]);
            std::cout<<"xyz "<<xyz<<std::endl;
            std::cout<<"offset "<<offset<<std::endl;
            std::cout<<"mconfig.resolution "<<mconfig.resolution<<std::endl;
            voxelMapptr.reset(new voxel_map::VoxelMap(xyz, offset, mconfig.resolution));
            sdf_ptr.reset(new SDFMap(mconfig.esdf_vis_slice_height_));
            sdf_ptr->initMap(nh);
            jump_lib_ptr.reset(new jump_library(mconfig.robot_jump_highest,mconfig.robot_jump_v2d_max,mconfig.robot_radius));
            jump_lib_ptr->init(nh,sdf_ptr);
            mapSub = nh.subscribe(mconfig.subscribe_cloudTopic, 1, &UGVPlannerManager::mapCallBack, this,
                                  ros::TransportHints().tcpNoDelay());
            pub_surface_cloud1 = nh.advertise<visualization_msgs::Marker>(mconfig.pub_surface_grid_Topic1, 200000);
            pub_surface_cloud2 = nh.advertise<visualization_msgs::Marker>(mconfig.pub_surface_grid_Topic2, 200000);
            angle_color_cloud_pub_=  nh.advertise<sensor_msgs::PointCloud2>("/angle_color_cloud", 10);
            pub_cloud_what_you_Want=  nh.advertise<sensor_msgs::PointCloud2>("/cloud___", 10);
            target_sub     = nh.subscribe(mconfig.sub_goal, 1, &UGVPlannerManager::targetRcvCallback, this);
            odom_sub       = nh.subscribe("/model_odom",1 ,&UGVPlannerManager::odomRcvCallback, this);
            astar_path_vis_pub   = nh.advertise<visualization_msgs::Marker>("astar_path_vis", 1);
            path_For_tracking_pub = nh.advertise<nav_msgs::Path>("trajectory_for_tracking",10);
            path_For_tracking_vis_pub = nh.advertise<nav_msgs::Odometry>("trajectory_for_tracking_vis",10);
            traj_pub      = nh.advertise<mpc::Polynome>("trajectory",3);
        }

    void odomRcvCallback(const nav_msgs::Odometry odom)//从simulator那来的，暂时先注释掉一部分
    {
        if(has_odom == false){
            ROS_INFO("Get odometry");
            has_odom = true;
        }
        
        now_pos(0) = odom.pose.pose.position.x;
        now_pos(1) = odom.pose.pose.position.y;
        now_pos(2) = odom.pose.pose.position.z;

        if(init_all == true){
        Eigen::Vector3i now_index;
        now_index=voxelMapptr->posD2I(now_pos);
        for(int z=now_index(2);z>=0;z--)
        {
            Eigen::Vector3i temp_index=now_index;
            temp_index(2)=z;
            int tmp_=temp_index.dot(voxelMapptr->getstep());
            std::vector<uint8_t> tmp_voxel=voxelMapptr->getVoxels();
            if (tmp_voxel[tmp_] == 2)
            {
                now_index(2)=temp_index(2);
                // std::cout<<"drop_on_surface"<<std::endl;
                break;
            }
        }
        now_pos=voxelMapptr->posI2D(now_index);
        }

    }
    void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!get_map)
        {
            get_map = true;
            
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
            voxelMapptr->setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                        fdata[cur + 1],
                                                        fdata[cur + 2]));
            }
        //query的时候OOC和Dilated都是true，要在还没有dilate之前把esdf建立完
            sdf_ptr->setEnvironment(voxelMapptr);
            voxelMapptr->dilate();
            std::vector<Eigen::Vector3d> surface_points;
            voxelMapptr->getSurf(surface_points);


            // //很操蛋，如果用esdf来做表面的法线，因为离散化的原因，注定会出现对于土丘这样的表面
            // //满足角度倾角的栅格本应该连续却不连续了。。
            // //还是得用PCA的方法
            //不仅如此，斜坡那里问题也很严重
            ros::Time T1=ros::Time::now();
            pcl::PointCloud<pcl::PointXYZ>::Ptr PCAcloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointXYZ pt_surface;
            //现在的拟法是用表面上栅格化的点拟,我想不栅格化拟表面，这样点多一些，以及雷达扫出来的点天然的就在表面
            //我觉得雷达建图建完之后，不用找表面点了，直接PCA拟合，就是得要求建完的这个图别栅格化，最好稠密一点？
            for(int tmp=0;tmp<surface_points.size();tmp++){
                pt_surface.x=surface_points[tmp](0);
                pt_surface.y=surface_points[tmp](1);
                pt_surface.z=surface_points[tmp](2);
                PCAcloud->points.push_back(pt_surface);
            }
            std::cout<<"surface_points.size() "<<surface_points.size()<<std::endl;
            // visualize(surface_points,0);
            // pcl::fromROSMsg(*msg,*PCAcloud);
            pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
            ne.setInputCloud(PCAcloud);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> );
            // pcl::KdTreeFLANN<pcl::PointXYZ> tree;

            ne.setSearchMethod(tree);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
            ne.setRadiusSearch(mconfig.robot_radius);
            ne.compute(*cloud_normals);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields (*PCAcloud, *cloud_normals, *cloud_with_normals);
            ros::Time T2=ros::Time::now();
            ros::Duration delta_T1=T2-T1;
            std::cout<<"计算法线 "<<delta_T1<<std::endl;


        T1=ros::Time::now();
        //如果一个点周围找不到足够的点计算平面，则法线中会赋值为nan
        std::vector<Eigen::Vector3d> manifold_edge_set;
        std::vector<Eigen::Vector3d> surface_points_after_angle_filter_positive;
        std::vector<Eigen::Vector3d> angle_height_ok_set;
        std::cout<<"cloud_with_normals->points.size() "<<cloud_with_normals->points.size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZI> angle_color_cloud;
        for(int tmp=0;tmp<cloud_with_normals->points.size();tmp++)
        {
            if(
            isnan(cloud_with_normals->points[tmp].normal_x)||
            isnan(cloud_with_normals->points[tmp].normal_y)||
            isnan(cloud_with_normals->points[tmp].normal_z))
            {
            continue;
            }
            //获得该点法线
            Eigen::Vector3d pos_tmp;
            pos_tmp(0)=cloud_with_normals->points[tmp].x;
            pos_tmp(1)=cloud_with_normals->points[tmp].y;
            pos_tmp(2)=cloud_with_normals->points[tmp].z;
            Eigen::Vector3d grad_tmp,grad_esdf;
            grad_tmp(0)=cloud_with_normals->points[tmp].normal_x;
            grad_tmp(1)=cloud_with_normals->points[tmp].normal_y;
            grad_tmp(2)=cloud_with_normals->points[tmp].normal_z;


       
            //要求pcl算出来的法线方向和esdf的一致
            double cost_tmp;
            cost_tmp=sdf_ptr->getDistWithGradTrilinear(pos_tmp, grad_esdf);
            double cos_value=grad_tmp.dot(grad_esdf)/(grad_tmp.norm()*grad_esdf.norm());
            if (cos_value<0)//如果不同向
           {
grad_tmp=-grad_tmp;
           }     

            //计算法线倾斜度是否ok
            Eigen::Vector3d z_axis(0,0,1);
            double cosvalue=grad_tmp.dot(z_axis)/(grad_tmp.norm()*z_axis.norm());
            double angle=acos(cosvalue)*180/3.1415926;
            if(angle<40){
               


                    bool if_z_is_ok=true;
                    for(int check_z_occ=1;check_z_occ<ceil(mconfig.robot_height_min/mconfig.resolution);check_z_occ++)
                    {
                        Eigen::Vector3i idx_tmp=voxelMapptr->posD2I(pos_tmp);
                        Eigen::Vector3i idx_tmp_z=idx_tmp;
                        idx_tmp_z(2)+=check_z_occ;
                        if(voxelMapptr->query(idx_tmp_z)==true)
                        {
                            Eigen::Vector3d pos_tmp_z=voxelMapptr->posI2D(idx_tmp_z);
                            if_z_is_ok=false;
                        
                        }
                            
                    }
                    if(if_z_is_ok==true)//看垂直高度上是否不occ
                    {
                        //  vis_grad(pos_tmp, grad_tmp,tmp);
                        angle_height_ok_set.push_back(pos_tmp);
                        voxelMapptr->set_angle_height_ok(pos_tmp);
                        // ros::Duration(0.01).sleep();
                    }
                    else{
                    }
            }

            //根据angle不同发布不同颜色的点云

            pcl::PointXYZI pt;
            pt.x = pos_tmp(0);
            pt.y = pos_tmp(1);
            pt.z = pos_tmp(2);
            pt.intensity =1-angle/180;
            angle_color_cloud.push_back(pt);
                

        }
        std::cout<<"angle_height_ok_set.size()"<<angle_height_ok_set.size()<<std::endl;
        angle_color_cloud.width = angle_color_cloud.points.size();
        angle_color_cloud.height = 1;
        angle_color_cloud.is_dense = true;
        angle_color_cloud.header.frame_id = "world";
        sensor_msgs::PointCloud2 angle_color_cloud_msg;
        pcl::toROSMsg(angle_color_cloud, angle_color_cloud_msg);
        angle_color_cloud_pub_.publish(angle_color_cloud_msg);
        T2=ros::Time::now();
        delta_T1=T1-T2;
        std::cout<<"check 法线和倾角 "<<delta_T1<<std::endl;


        // visualize(angle_height_ok_set,1);


///---------------下面找倾角和上方free的区域的边缘edge以便膨胀出安全距离
        //打算用的找流形边缘的方式1是DON，看法线微分    
        //2是我想试一下，栅格化后，找相邻26个点中那些在满足倾斜度和上限高度的集合里 投影到2d上是否把周围把自己包围住了
        //这个包住并不是8格包住才行，其实东南西北4个包就ok   
        //且得是先限制高度再限制膨胀，不然会出现悬空的很低的天花板投影边缘没有安全距离
        //check26要遍历,但法线微分也要遍历，check26在处理边的连通关系也要用到
        for(int tmp=0 ;tmp<angle_height_ok_set.size();tmp++)
        {
            Eigen::Vector3d pos_now=angle_height_ok_set[tmp];
            Eigen::Vector3i pos_now_index=voxelMapptr->posD2I(pos_now);
            bool if_has_have_all_surrouned=true;//有一个false就说明包围不完全
            for(int x=-1;x<=1;x++)
            {
                for(int y=-1;y<=1;y++)
                {   
                    bool if_this_pair_of_xy_has_ok=false;//有一个true就算这个xy ok
                    for(int z=-1;z<=1;z++)
                    {
                        if((x==0)&&(y==0)&&(z==0))
                            continue;
                        Eigen::Vector3i bias(x,y,z);
                        Eigen::Vector3i index_neighbor;
                        index_neighbor=pos_now_index+bias;
                        if(voxelMapptr->query_if_angle_height_ok(index_neighbor))
                        { 
                            if_this_pair_of_xy_has_ok=true;
                            voxelMapptr->set_connect_information_without_dilate(pos_now,index_neighbor);
                        }
                    }
                    if((if_this_pair_of_xy_has_ok==false)&&((abs(x)+abs(y))==1))
                        if_has_have_all_surrouned=false;//且没所谓顶角和中心点有东西，   边角就行
                }
            }
            if(if_has_have_all_surrouned==false)
                manifold_edge_set.push_back(pos_now);
        }
        // visualize(manifold_edge_set,0);
        std::cout<<"manifold_edge_set.size()"<<manifold_edge_set.size()<<std::endl;
        //根据流形边缘膨胀,因为确定倾角的pca方法会导致滤波更保守，所以这里就用floor好了
        int corrode_cellnum =floor(mconfig.robot_radius/mconfig.resolution)-1;
        std::cout<<"corrode_cellnum"<<corrode_cellnum<<std::endl;
        for(int tmp=0;tmp<manifold_edge_set.size();tmp++)
        {
            Eigen::Vector3i pos_now_index=voxelMapptr->posD2I(manifold_edge_set[tmp]);
            for(int x=-corrode_cellnum;x<=corrode_cellnum;x++)
            for(int y=-corrode_cellnum;y<=corrode_cellnum;y++)
            for(int z=-corrode_cellnum;z<=corrode_cellnum;z++)
            {
                Eigen::Vector3i bias(x,y,z);
                Eigen::Vector3i index_neighbor;
                index_neighbor=pos_now_index+bias;
                //把边缘的这些个邻居，他们的最终可行信息置为not_ok;
                voxelMapptr->set_end_feasible_not_ok(index_neighbor);
            }
        }


        for(int x=0;x<=(mconfig.mapBound[1] - mconfig.mapBound[0]) / mconfig.resolution;x++)
        for(int y=0;y<=(mconfig.mapBound[3] - mconfig.mapBound[2]) / mconfig.resolution;y++)
        for(int z=0;z<=(mconfig.mapBound[5] - mconfig.mapBound[4]) / mconfig.resolution;z++)
        {
            Eigen::Vector3i id(x,y,z);
            Eigen::Vector3d pos_tmp=voxelMapptr->posI2D(id);
            if(voxelMapptr->query_if_end_feasible_ok(id))
                end_feasible_set.push_back(pos_tmp);
        }
        std::cout<<"end_feasible_set.size()"<<end_feasible_set.size()<<std::endl;
        // visualize(end_feasible_set,1);
///---------------下面找最终可行域的边缘edge
        T1=ros::Time::now();

        delta_T1=T1-T2;
        std::cout<<"找满足倾角和高度ok的边缘+set联通信息 "<<delta_T1<<std::endl;
        for(int tmp=0 ;tmp<end_feasible_set.size();tmp++)
        {
            Eigen::Vector3d pos_now=end_feasible_set[tmp];
            Eigen::Vector3i pos_now_index=voxelMapptr->posD2I(pos_now);
            bool if_has_have_all_surrouned=true;//有一个false就说明包围不完全
            for(int x=-1;x<=1;x++)
            {
                for(int y=-1;y<=1;y++)
                {   
                    bool if_this_pair_of_xy_has_ok=false;//有一个true就算这个xy ok
                    for(int z=-1;z<=1;z++)
                    {
                        if((x==0)&&(y==0)&&(z==0))
                            continue;
                        Eigen::Vector3i bias(x,y,z);
                        Eigen::Vector3i index_neighbor;
                        index_neighbor=pos_now_index+bias;
                        if(voxelMapptr->query_if_end_feasible_ok(index_neighbor))
                        { 
                            if_this_pair_of_xy_has_ok=true;
                            voxelMapptr->set_connect_information_on_end_feasible(pos_now,index_neighbor);
                        }
                    }
                    if((if_this_pair_of_xy_has_ok==false)&&((abs(x)+abs(y))==1))
                        if_has_have_all_surrouned=false;//且没所谓顶角和中心点有东西，   边角就行
                }
            }
            if(if_has_have_all_surrouned==false)
                end_feasible_edge_set.push_back(pos_now);
        }
        std::cout<<"end_feasible_edge_set.size()"<<end_feasible_edge_set.size()<<std::endl;
        
        T2=ros::Time::now();
        delta_T1=T2-T1;
        std::cout<<"找最终可行域的边缘edge和设置可行域联通信息花费时间"<<delta_T1<<std::endl;

        voxelMapptr->init_GridNodeMap();
        voxelMapptr->caculate_end_Feasible_surf(ceil(mconfig.robot_height_min/mconfig.resolution),ceil(mconfig.robot_height_max/mconfig.resolution));
        voxelMapptr->get_end_feasible_Surf(end_Feasible_surf);
        std::cout<<"end_Feasible_surf "<<end_Feasible_surf.size()<<std::endl; 
        // visualize(end_Feasible_surf,1);
        voxelMapptr->construct_disjoint_Set();

//下面这些很浪费时间  只是显示用的
        vector<int> big_father_Set;
        for(int x=0;x<(mconfig.mapBound[1] - mconfig.mapBound[0]) / mconfig.resolution;x++)
        for(int y=0;y<(mconfig.mapBound[3] - mconfig.mapBound[2]) / mconfig.resolution;y++)
        for(int z=0;z<(mconfig.mapBound[5] - mconfig.mapBound[4]) / mconfig.resolution;z++)
                {
                    Eigen::Vector3i current_index(x,y,z);
                    int tmp=current_index.dot(voxelMapptr->getstep());
                    if(voxelMapptr->find_Father_until_root(tmp)!=tmp)// 
                    {
                        int this_Father=voxelMapptr->find_Father_until_root(tmp);
                        // std::cout<<"this_Father"<<this_Father<<std::endl;
                        bool this_Father_is_repeat=false;
                        for(int q=0;q<big_father_Set.size();q++){
                            if(this_Father==big_father_Set[q])
                                this_Father_is_repeat=true;
                        }
                        if(this_Father_is_repeat==false)
                            big_father_Set.push_back(voxelMapptr->find_Father_until_root(tmp));
                            
                    }
            }
                        std::cout<<"big_father_Set_size"<<big_father_Set.size()<<std::endl;
                        for(int q=0;q<big_father_Set.size();q++){
                        std::cout<<"big_father_"<<big_father_Set[q]<<std::endl;
                        }

        init_all=true;

        


        
        }
        else{
                // visualize(end_feasible_set,0);
                visualize(end_feasible_edge_set,1);
                pub_cloud(end_Feasible_surf); 
        }
    
    }
    void targetRcvCallback(const geometry_msgs::PoseStamped target_info)
    { 
        if((!init_all)  ||  (!has_odom))
            return;
        Eigen::Vector3d target_pos;     
        target_pos(0) = target_info.pose.position.x;
        target_pos(1) = target_info.pose.position.y;
        target_pos(2) = target_info.pose.position.z+0.5;

        Eigen::Vector3i target_index;
        target_index=voxelMapptr->posD2I(target_pos);
        for(int z=target_index(2);z>=0;z--)
        {
            Eigen::Vector3i temp_index=target_index;
            temp_index(2)=z;
            //把目标点从上往下落，落到表面上为止  id.dot(step)
            int tmp_=temp_index.dot(voxelMapptr->getstep());
            std::vector<uint8_t> tmp_voxel=voxelMapptr->getVoxels();
            if (tmp_voxel[tmp_] == 2)
            {
                target_index(2)=temp_index(2);
                std::cout<<"drop_on_surface"<<std::endl;
                break;
            }
        }



        target_pos=voxelMapptr->posI2D(target_index);
std::cout<<"now_pos "<<now_pos<<std::endl;
        Eigen::Vector3i pos_index=voxelMapptr->posD2I(now_pos);
        int if_start_angle_height_ok=voxelMapptr->query_if_angle_height_ok(pos_index);
        std::cout<<"if_start_angle_height_ok "<<if_start_angle_height_ok<<std::endl;
        int if_start_feasible=voxelMapptr->query_if_end_feasible_ok(pos_index);
        std::cout<<"if_start_feasible "<<if_start_feasible<<std::endl;

        if(if_start_feasible==false)
            ROS_ERROR("start_not_Feasible");   

std::cout<<"target_pos "<<target_pos<<std::endl;
        Eigen::Vector3i target_index_cout=voxelMapptr->posD2I(target_pos);
        int if_end_angle_height_ok=voxelMapptr->query_if_angle_height_ok(target_index_cout);
        std::cout<<"if_end_angle_height_ok "<<if_end_angle_height_ok<<std::endl;
        int if_end_feasible=voxelMapptr->query_if_end_feasible_ok(target_index_cout);
        std::cout<<"if_end_feasible "<<if_end_feasible<<std::endl;

        if(if_end_feasible==false)
            ROS_ERROR("end_not_Feasible");   

        //esdf
        double cost_tmp;            Eigen::Vector3d grad_esdf;
        cost_tmp=sdf_ptr->getDistWithGradTrilinear(target_pos, grad_esdf);
        vis_grad(target_pos, grad_esdf,1);



        ros::Time T1=ros::Time::now();
        vector<Eigen::Vector3d> astar_path= voxelMapptr->AstarnodeSearch(now_pos,target_pos);
        
        visastarPath(astar_path);
        ros::Time T2=ros::Time::now();
        ros::Duration delta_T1=T2-T1;
        std::cout<<"astar cost time"<<delta_T1<<std::endl;
        std::vector<Eigen::MatrixX4d> hPolys;
        T1=ros::Time::now();
        sfc_gen::convexCover(astar_path,
                                end_Feasible_surf,
                                voxelMapptr->getOrigin(),
                                voxelMapptr->getCorner(),//7 3
                                4.0,
                                2.0,
                                hPolys);
        T2=ros::Time::now();
        delta_T1=T2-T1;
        std::cout<<"delta_T1"<<delta_T1<<std::endl;
        sfc_gen::shortCut(hPolys);
        ros::Time T3=ros::Time::now();
        ros::Duration delta_T2=T3-T2;
        std::cout<<"delta_T2"<<delta_T2<<std::endl;
        visualizer.visualizePolytope(hPolys);

        Eigen::Matrix3d iniState;
        Eigen::Matrix3d finState;
        Eigen::Vector3d start_For_minco=astar_path.front();
        start_For_minco(2)+=0.1;//0.511;
        Eigen::Vector3d end_For_minco=astar_path.back();
        end_For_minco(2)+=0.1;//0.511;
        iniState << start_For_minco, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        finState << end_For_minco, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        gcopter::GCOPTER_PolytopeSFC gcopter;
        Eigen::VectorXd magnitudeBounds(5);
        Eigen::VectorXd penaltyWeights(5);
        Eigen::VectorXd physicalParams(6);
        magnitudeBounds(0) = mconfig.maxVelMag;
        magnitudeBounds(1) = mconfig.maxBdrMag;
        magnitudeBounds(2) = mconfig.maxTiltAngle;
        magnitudeBounds(3) = mconfig.minThrust;
        magnitudeBounds(4) = mconfig.maxThrust;
        penaltyWeights(0) = (mconfig.chiVec)[0];
        penaltyWeights(1) = (mconfig.chiVec)[1];
        penaltyWeights(2) = (mconfig.chiVec)[2];
        penaltyWeights(3) = (mconfig.chiVec)[3];
        physicalParams(0) = mconfig.vehicleMass;
        physicalParams(1) = mconfig.gravAcc;
        physicalParams(2) = mconfig.horizDrag;
        physicalParams(3) = mconfig.vertDrag;
        physicalParams(4) = mconfig.parasDrag;
        physicalParams(5) = mconfig.speedEps;
        const int quadratureRes = mconfig.integralIntervs;

        traj.clear();

        if (!gcopter.setup(mconfig.weightT,
                            iniState, finState,
                            hPolys, INFINITY,
                            mconfig.smoothingEps,
                            quadratureRes,
                            magnitudeBounds,
                            penaltyWeights,
                            physicalParams))
        {
            return;
        }

        if (std::isinf(gcopter.optimize(traj, mconfig.relCostTol)))
        {
            return;
        }

        if (traj.getPieceNum() > 0)
        {
            trajStamp = ros::Time::now().toSec();
            visualizer.visualize(traj, astar_path);
            publishTraj(traj);
        }
        // if (traj.getPieceNum() > 0)
        // {
        //             nav_msgs::Odometry traj_odom;
        //             nav_msgs::Path path;
        //             path.header.stamp=traj_odom.header.stamp=ros::Time::now();
        //             path.header.frame_id=traj_odom.header.frame_id="world";
                    
        //             ros::Time current_time = ros::Time::now();
            
        //             double T = 0.01;
        //             for (double t = 0; t < traj.getTotalDuration(); t += T)
        //             {   
        //                 Eigen::Vector3d X = traj.getPos(t);
        //                 geometry_msgs::PoseWithCovariance this_pose_stamped;
        //                 geometry_msgs::PoseStamped path_stamped_point;
        //                 this_pose_stamped.pose.position.x = X(0);
        //                 this_pose_stamped.pose.position.y = X(1);
        //                 this_pose_stamped.pose.position.z = X(2);

        // //算一下落地的foot，  path发的是头部顶端轨迹，需要给出脚步轨迹，其实就是把这个点的脚步的grou_hight 给出来就行
        //                 Eigen::Vector3i foot_index;
        //                 Eigen::Vector3d foot_pos=traj.getPos(t);
        //                 foot_index=voxelMapptr->posD2I(foot_pos);
        //                 for(int z=foot_index(2);z>=0;z--)
        //                 {
        //                     Eigen::Vector3i temp_index=foot_index;
        //                     temp_index(2)=z;
        //                     int tmp_=temp_index.dot(voxelMapptr->getstep());
        //                     std::vector<uint8_t> tmp_voxel=voxelMapptr->getVoxels();
        //                     if (tmp_voxel[tmp_] == 2)
        //                     {
        //                         foot_index(2)=temp_index(2);
        //                         break;
        //                     }
        //                 }
        //                 //妈的 为什么给我tp
        //                 foot_pos=voxelMapptr->posI2D(foot_index);



        //                 Eigen::Vector3d V=traj.getVel(t);
        //                 V(2)=0;
        //                 double dim_2_sqrtnorm=V[0]*V[0]+V[1]*V[1];
        //                 if(sqrt(dim_2_sqrtnorm)>0.2){//我想把首尾方向乱掉的那几个点给扔了...可能是因为到那速度基本为0了？导致乱掉了
                    
        //                 Eigen::AngleAxisd t_v(atan2(V[1],V[0]),Vector3d(0,0,1));
        //                 Eigen::Quaterniond rot(t_v);
        //                 this_pose_stamped.pose.orientation.x = rot.x();
        //                 this_pose_stamped.pose.orientation.y = rot.y();
        //                 this_pose_stamped.pose.orientation.z = rot.z();
        //                 this_pose_stamped.pose.orientation.w = rot.w();

        //                 traj_odom.pose=this_pose_stamped;
        //                 // path_For_tracking_vis_pub.publish(traj_odom);
                        
        //                 path_stamped_point.header.seq=foot_pos(2)*100;
        //                 path_stamped_point.pose=this_pose_stamped.pose;
        //                 path.poses.push_back(path_stamped_point);
        //                 // ros::Duration(0.01).sleep();
        //                 }
        //             }

        //             path_For_tracking_pub.publish(path);

        //     }
        voxelMapptr->resetUsedGrids();  
    }
    void publishTraj(const Trajectory<5> &traj)
{
    mpc::Polynome poly;
    // if (traj.getPieceNum() > 0)
    // {
    //     double T = 0.1;
    //     for (double t = T; t < traj.getTotalDuration(); t += T)
    //     {
    //         geometry_msgs::Point point;
    //         Eigen::Vector3d X = traj.getPos(t);
    //       double v= traj.getVel(t).norm();
    //       std::cout<<"refer v"<<v <<std::endl;
    //         point.x = X(0);
    //         point.y = X(1);
    //         point.z = X(2);
    //         poly.pos_pts.push_back(point);
    //         poly.t_pts.push_back(t);
    //     }

    // }
    MatrixXd poses = traj.getPositions();
    VectorXd ts    = traj.getDurations();

    for (int i = 0; i < poses.cols(); i++)
    {
      geometry_msgs::Point temp;
      temp.x = poses(0, i);
      temp.y = poses(1, i);
      temp.z = poses(2, i);
      poly.pos_pts.push_back(temp);
    }
    for (int i = 0; i < ts.size(); i++)
    {
      poly.t_pts.push_back(ts(i));
      double v= traj.getVel(ts(i)).norm();
      std::cout<<"refer v"<<v <<std::endl;
    }

    poly.init_v.x = 0;
    poly.init_v.y = 0;
    poly.init_v.z = 0;
    poly.init_a.x = 0;
    poly.init_a.y = 0;
    poly.init_a.z = 0;
    poly.start_time = ros::Time::now();

    traj_pub.publish(poly);
    std::cout<<"pub poly"<<std::endl;
}

    void visualize( std::vector<Eigen::Vector3d> &surface,int color_type)
    {
        visualization_msgs::Marker surfaceMarker;

        surfaceMarker.id = 0;
        surfaceMarker.type = visualization_msgs::Marker::CUBE_LIST;
        surfaceMarker.header.stamp = ros::Time::now();
        surfaceMarker.header.frame_id = "world";
        surfaceMarker.pose.orientation.w = 1.00;
        surfaceMarker.action = visualization_msgs::Marker::ADD;
        surfaceMarker.ns = "route";
        if(color_type==1){
            surfaceMarker.color.r = 1.00;
            surfaceMarker.color.g = 0.00;
            surfaceMarker.color.b = 1.00;
        surfaceMarker.color.a = 10;
        surfaceMarker.scale.x = 0.1;
        surfaceMarker.scale.y = 0.1;
        surfaceMarker.scale.z = 0.1;
        }
        else{
            surfaceMarker.color.r = 0.00;
            surfaceMarker.color.g = 0.00;
            surfaceMarker.color.b = 1.00;
        surfaceMarker.color.a = 10;
        surfaceMarker.scale.x =  0.1;
        surfaceMarker.scale.y =  0.1;
        surfaceMarker.scale.z =  0.1;
        }



        
        if (surface.size() > 0)
        {
            bool first = true;
            for (auto it : surface)
            {
                geometry_msgs::Point point;
                point.x = it(0);
                point.y = it(1);
                point.z = it(2);
                surfaceMarker.points.push_back(point);

            }
            if(color_type==1)
                pub_surface_cloud1.publish(surfaceMarker);
            else
                pub_surface_cloud2.publish(surfaceMarker);
        }
    }
    void vis_grad(Eigen::Vector3d pos, Eigen::Vector3d& grad, int id)
    {
            visualization_msgs::Marker sphere;
            sphere.header.frame_id  = "world";
            sphere.header.stamp     = ros::Time::now();
            sphere.type             = visualization_msgs::Marker::ARROW;
            sphere.action           = visualization_msgs::Marker::ADD;
            sphere.id               = id;


            sphere.color.r              = 0.0;
            sphere.color.g              = 1;
            sphere.color.b              = 0.0;
            sphere.color.a              = 1;
            sphere.scale.x              = 0.3;//grad.norm();
            sphere.scale.y              = 0.025;
            sphere.scale.z              = 0.025;
            sphere.pose.position.x      = pos(0);
            sphere.pose.position.y      = pos(1);
            sphere.pose.position.z      = pos(2);

            Vector3d A(1,0,0);
            Vector3d B=A.cross(grad);
            B.normalize();
            Eigen::AngleAxisd t_v(acos(A.dot(grad)/grad.norm()/A.norm()),B);


            Eigen::Quaterniond rot(t_v);
            rot.normalize();
            // cout<<"rot.x  "<<rot.x()<<"  rot.y  "<<rot.y()<<"  rot.z  "<<rot.z()<<"  rot.w  "<<rot.w()<<endl;
            sphere.pose.orientation.x      = rot.x();
            sphere.pose.orientation.y      = rot.y();
            sphere.pose.orientation.z      = rot.z();
            sphere.pose.orientation.w      = rot.w();
            // sphere.lifetime = ros::Duration(100);
            sdf_ptr->grad_pub.publish(sphere);
    }
    void pub_cloud( std::vector<Eigen::Vector3d> &your_Vector3d)
    { 
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ pt;

         if (your_Vector3d.size() > 0)
        {
            for (auto it : your_Vector3d)
            {
                pt.x = it(0);
                pt.y = it(1);
                pt.z = it(2);
                cloud.push_back(pt);
            }

        }
        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "world";
        sensor_msgs::PointCloud2 _cloud_msg;
        pcl::toROSMsg(cloud, _cloud_msg);
        pub_cloud_what_you_Want.publish(_cloud_msg);
    }
    void visastarPath( vector<Vector3d> nodes)
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();


    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;


    node_vis.color.a = 1.0;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;



    node_vis.scale.x = mconfig.resolution;
    node_vis.scale.y = mconfig.resolution;
    node_vis.scale.z = mconfig.resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    astar_path_vis_pub.publish(node_vis);
}
};




const double pi = 3.1415926535;
int main(int argc, char **argv)
{

  ros::init(argc, argv, "ugv_planner_node");
  ros::NodeHandle nh("~");

  UGVPlannerManager main_planner(mapConfig(ros::NodeHandle("~")), nh);
  
  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
