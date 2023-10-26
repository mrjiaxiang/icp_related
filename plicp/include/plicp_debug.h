#ifndef PLICP_DEBUG_H
#define PLICP_DEBUG_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include "iterative_closest_point.h"
#include "plicp.h"
#include "pose2d.h"

struct LaserLine{
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    float end_dis;
    float end_edg;
};

struct LidarDataInfo{
    uint64_t timestamp;
    Pose2D odo_pose;
    std::vector<LaserLine> laser_lines;
};

class PLICPDebug {
 public:
    PLICPDebug();

    //将激光消息转换为激光坐标系下的二维点云
    void ConvertLaserScanToEigenPointCloud(const LidarDataInfo& msg, std::vector<Eigen::Vector2d>& eigen_pts);

    void LaserScanCallback(const LidarDataInfo& msg);

    bool m_isFirstFrame;
    PLICPMatcher m_plicpMatcher;
    Eigen::Vector3d m_prevLaserPose;
    std::vector<Eigen::Vector2d> m_prevPointCloud;
    std::vector<Eigen::Vector2d> m_icp_prevPointCloud;
    std::vector<Eigen::Vector2d> m_icp_currPointCloud;
    std::vector<Pose2D> m_odom;
    Pose2D m_cur_odom;
    Pose2D m_pre_odom;
    Pose2D m_trans;
    std::vector<std::pair<Pose2D,Pose2D>> odom_and_scanOdom;
    IterativeClosestPoint m_icp;
};

#endif