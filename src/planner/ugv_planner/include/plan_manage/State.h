#ifndef _STATE_H_
#define _STATE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

struct TrajectoryState;
typedef TrajectoryState* TrajectoryStatePtr;
//z轴上我还是不用速度去离散了，不跳的时候z轴都是靠头伸缩搞出来的，跳跃之前的轨迹末状态z的速度没卵用
//但是径向上的速度需要输入（匀速的），跳跃之前的轨迹末状态的速度决定了能跳多远

struct TrajectoryState
{
    std::vector<Eigen::Vector3d> Position;
    double Trajctory_Cost ;
    bool collision_check ;           //False -> no collision, True -> collision

    TrajectoryState(std::vector<Eigen::Vector3d> _Position){
        Position        = _Position;
        collision_check = false;
    }
    TrajectoryState(){};
    ~TrajectoryState(){};

    void setCollision(){
        collision_check = true;
    }
};

#endif