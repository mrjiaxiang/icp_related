#include "plicp_debug.h"
#include "time.h"

PLICPDebug::PLICPDebug() {
    m_isFirstFrame = true;
}

//将激光消息转换为激光坐标系下的二维点云
void PLICPDebug::ConvertLaserScanToEigenPointCloud(const LidarDataInfo& msg, std::vector<Eigen::Vector2d>& eigen_pts) {
    eigen_pts.clear();
    for (int i = 0; i < msg.laser_lines.size(); ++i) {
        if (msg.laser_lines[i].end_dis < 0.1 || msg.laser_lines[i].end_dis > 8.0)
            continue;

        double lx = msg.laser_lines[i].end_x;
        double ly = msg.laser_lines[i].end_y;

        if (std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly))
            continue;

        eigen_pts.push_back(Eigen::Vector2d(lx, ly));
    }
}

void PLICPDebug::LaserScanCallback(const LidarDataInfo& msg)
{
    if (m_isFirstFrame == true) {
        std::cout << "First Frame" << std::endl;
        m_isFirstFrame = false;
        m_odom.push_back(msg.odo_pose);
        m_pre_odom      = msg.odo_pose;
        m_prevLaserPose = Eigen::Vector3d(0,0,0);
        ConvertLaserScanToEigenPointCloud(msg,m_icp_prevPointCloud);
        ConvertLaserScanToEigenPointCloud(msg, m_prevPointCloud);
        return;
    }

    std::vector<Eigen::Vector2d> nowPts;
    std::vector<Eigen::Vector2d> nowICPPts;
    m_odom.push_back(msg.odo_pose);
    m_cur_odom = msg.odo_pose;
    ConvertLaserScanToEigenPointCloud(msg, nowPts);
    ConvertLaserScanToEigenPointCloud(msg, nowICPPts);
//    std::cout<<nowPts.size()<<std::endl;

    Pose2D odom_estimate = m_cur_odom - m_pre_odom;
    //调用plicp进行icp匹配，并输出结果．
    m_plicpMatcher.setSourcePointCloud(nowPts);
    m_plicpMatcher.setTargetPointCloud(m_prevPointCloud);

    Eigen::Matrix3d rPose, rCovariance;
    // cout << " num: " << count++ << endl;
    if (m_plicpMatcher.Match(rPose, rCovariance)) {
        // std::cout << "---2---" << std::endl;
        std::cout << "plicp Match Successful:" << rPose(0, 2) << "," << rPose(1, 2) << ","
                  << atan2(rPose(1, 0), rPose(0, 0)) << std::endl;
        Eigen::Matrix3d lastPose;
        lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0), sin(m_prevLaserPose(2)),
                cos(m_prevLaserPose(2)), m_prevLaserPose(1), 0, 0, 1;
        Eigen::Matrix3d nowPose = lastPose * rPose;
        m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1, 0), nowPose(0, 0));
    } else {
        std::cout << "PLICP Match Failed!!!!" << std::endl;
        float nowPts_x[360] = {0};
        float nowPts_y[360] = {0};
        for (int i = 0; i < nowICPPts.size(); i++) {
            nowPts_x[i] = nowICPPts[i](0, 0);
            nowPts_y[i] = nowICPPts[i](1, 0);
        }

        float prePts_x[360] = {0};
        float prePts_y[360] = {0};
        for (int i = 0; i < nowPts.size(); i++) {
            prePts_x[i] = m_icp_prevPointCloud[i](0, 0);
            prePts_y[i] = m_icp_prevPointCloud[i](1, 0);
        }
        m_icp.SetInputSource(nowPts_x, nowPts_y, 360);
        m_icp.SetInputTarget(prePts_x, prePts_x, 360);
        m_icp.Align(odom_estimate);
        m_trans = m_icp.GetFinalTransformation();
        double c_cos   = cos(m_trans.Phi());
        double c_sin   = sin(m_trans.Phi());
        rPose.setZero();
        rPose(0, 0) = c_cos;
        rPose(0, 1) = -c_sin;
        rPose(0, 2) = m_trans.x;
        rPose(1, 0) = c_sin;
        rPose(1, 1) = c_cos;
        rPose(1, 2) = m_trans.y;
        rPose(2, 2) = 1;
        std::cout << "icp Match Successful:" << rPose(0, 2) << "," << rPose(1, 2) << ","
                  << atan2(rPose(1, 0), rPose(0, 0)) << std::endl;
        Eigen::Matrix3d lastPose;
        lastPose << cos(m_prevLaserPose(2)), -sin(m_prevLaserPose(2)), m_prevLaserPose(0), sin(m_prevLaserPose(2)),
                cos(m_prevLaserPose(2)), m_prevLaserPose(1), 0, 0, 1;
        Eigen::Matrix3d nowPose = lastPose * rPose;
        m_prevLaserPose << nowPose(0, 2), nowPose(1, 2), atan2(nowPose(1, 0), nowPose(0, 0));
    }
    Pose2D scan_odom_estimate;
    scan_odom_estimate.x=rPose(0, 2);
    scan_odom_estimate.y=rPose(1, 2);
    scan_odom_estimate.SetPhi(atan2(rPose(1, 0), rPose(0, 0)));
    odom_and_scanOdom.push_back(std::pair<Pose2D,Pose2D>(odom_estimate,scan_odom_estimate));
    m_prevPointCloud = nowPts;
    m_icp_prevPointCloud=nowICPPts;
    m_pre_odom       = m_cur_odom;
}