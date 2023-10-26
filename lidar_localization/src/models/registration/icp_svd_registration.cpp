/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "glog/logging.h"

#include "lidar_localization/models/registration/icp_svd_registration.hpp"

namespace lidar_localization {

ICPSVDRegistration::ICPSVDRegistration(
    const Json::Value& node
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    // parse params:
    float max_corr_dist = node["max_corr_dist"].asFloat();
    float trans_eps = node["trans_eps"].asFloat();
    float euc_fitness_eps = node["euc_fitness_eps"].asFloat();
    int max_iter = node["max_iter"].asInt();

    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

ICPSVDRegistration::ICPSVDRegistration(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) : input_target_kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>()) {
    SetRegistrationParam(max_corr_dist, trans_eps, euc_fitness_eps, max_iter);
}

bool ICPSVDRegistration::SetRegistrationParam(
    float max_corr_dist, 
    float trans_eps, 
    float euc_fitness_eps, 
    int max_iter
) {
    // set params:
    max_corr_dist_ = max_corr_dist;
    trans_eps_ = trans_eps;
    euc_fitness_eps_ = euc_fitness_eps;
    max_iter_ = max_iter;
    //euc_fitness_eps 还有一条收敛条件是均方误差和小于阈值， 停止迭代。
    //trans_eps 前一个变换矩阵和当前变换矩阵的差异小于阈值时，就认为已经收敛了，是一条收敛条件；
    //max_corr_dist_ 搜索距离???
    LOG(INFO) << "ICP SVD params:" << std::endl
              << "max_corr_dist: " << max_corr_dist_ << ", "
              << "trans_eps: " << trans_eps_ << ", "
              << "euc_fitness_eps: " << euc_fitness_eps_ << ", "
              << "max_iter: " << max_iter_ 
              << std::endl << std::endl;

    return true;
}

bool ICPSVDRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    input_target_kdtree_->setInputCloud(input_target_);

    return true;
}

bool ICPSVDRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {


    input_source_ = input_source;
    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
//    if(predict_pose !=Eigen::Matrix4f::Identity()){
//        pcl::transformPointCloud(*input_source_,*transformed_input_source,predict_pose);
//    }else{
//        *transformed_input_source = *input_source_;
//    }
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);
    // init estimation:
    transformation_.setIdentity();

    //
    // TODO: first option -- implement all computing logic on your own
    //
    // do estimation:
    // 寻找最近临点

    int curr_iter = 0;
    while (curr_iter < max_iter_) {

        CloudData::CLOUD_PTR tmp_input_source(new CloudData::CLOUD());
        pcl::transformPointCloud(*transformed_input_source,*tmp_input_source,transformation_);

        std::vector<Eigen::Vector3f> xs;
        std::vector<Eigen::Vector3f> ys;
        //因为icp最少三对点
        if(GetCorrespondence(tmp_input_source,xs,ys) < 3){
            break;
        }
        LOG(INFO)<<"GetCorrespondence";
        Eigen::Matrix4f Txy;
        GetTransform(xs,ys,Txy);
        LOG(INFO)<<"Txy : "<<Txy;
        if(!IsSignificant(Txy,trans_eps_)){
            break;
        }

        transformation_ = Txy * transformation_;
        ++curr_iter;
    }

    // set output:
    result_pose = transformation_ * predict_pose;


    Eigen::Quaternionf  quaternionf(result_pose.block<3,3>(0,0));
    quaternionf.normalize();
    Eigen::Vector3f  t  = result_pose.block<3,1>(0,3);
    result_pose.setIdentity();
    result_pose.block<3,3>(0,0) = quaternionf.toRotationMatrix();
    result_pose.block<3,1>(0,3) = t;

    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    return true;
}

size_t ICPSVDRegistration::GetCorrespondence(
    const CloudData::CLOUD_PTR &input_source, 
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys
) {

    const float MAX_CORR_DIST_SQR = max_corr_dist_ * max_corr_dist_;

    size_t num_corr = 0;

    // TODO: set up point correspondence
    LOG(INFO)<<input_source->points.size();
    for (size_t  i =0; i < input_source->points.size(); ++i){
        std::vector<int> index;
        std::vector<float> distance;

        input_target_kdtree_->nearestKSearch(input_source->points.at(i), 1, index,distance);
            if(index.size() == 0 || distance.size() == 0){
                continue;
            }

            if(distance.at(0) > MAX_CORR_DIST_SQR)
                continue;

            Eigen::Vector3f x(
                    input_target_->points.at(index.at(0)).x,
                    input_target_->points.at(index.at(0)).y,
                    input_target_->points.at(index.at(0)).z
            );
            Eigen::Vector3f y(
                    input_source->points.at(i).x,
                    input_source->points.at(i).y,
                    input_source->points.at(i).z
            );

            xs.push_back(x);
            ys.push_back(y);
            ++num_corr;

    }

    return num_corr;
}

void ICPSVDRegistration::GetTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();

    // TODO: find centroids of mu_x and mu_y:
    Eigen::Vector3f point_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f point_y = Eigen::Vector3f::Zero();
    for(size_t i = 0; i < N; ++i){
        point_x += xs.at(i);
        point_y += ys.at(i);
    }
    point_x /= N;
    point_y /= N;
    // TODO: build H:
    //去质心
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for(size_t i = 0; i < N; i++){
         H += (ys.at(i) - point_y) * (xs.at(i) - point_x).transpose();
    }
    // TODO: solve R:
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H,Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H,Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    //这里细心一点,写成U*V.transpose()了
    Eigen::Matrix3f R = V*U.transpose();
    // TODO: solve t:
    Eigen::Vector3f t = point_x - R * point_y;
    // TODO: set output:
    transformation_.setIdentity();
    transformation_.block<3,3>(0,0) = R;
    transformation_.block<3,1>(0,3) = t;
}

bool ICPSVDRegistration::IsSignificant(
    const Eigen::Matrix4f &transformation,
    const float trans_eps
) {
    // a. translation magnitude -- norm:
    float translation_magnitude = transformation.block<3, 1>(0, 3).norm();
    // b. rotation magnitude -- angle:
    float rotation_magnitude = fabs(
        acos(
            (transformation.block<3, 3>(0, 0).trace() - 1.0f) / 2.0f
        )
    );

    return (
        (translation_magnitude > trans_eps) || 
        (rotation_magnitude > trans_eps)
    );
}

} // namespace lidar_localization