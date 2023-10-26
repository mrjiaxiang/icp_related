/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "glog/logging.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

namespace lidar_localization {

SICPRegistration::SICPRegistration(
        const Json::Value& node
) :target_normals_(new pcl::PointCloud<pcl::Normal>()){
    // parse params:
    params_.p = node["p"].asFloat();
    params_.mu = node["mu"].asFloat();
    params_.alpha = node["alpha"].asFloat();
    params_.max_mu = node["max_mu"].asFloat();
    params_.max_icp = node["max_icp"].asInt();
    params_.max_outer = node["max_outer"].asInt();
    params_.max_inner = node["max_inner"].asInt();
    params_.stop = node["stop"].asFloat();
}

bool SICPRegistration::AddNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 pcl::PointCloud<pcl::Normal>::Ptr normals){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud ( cloud );

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud ( cloud );
    normalEstimator.setSearchMethod ( searchTree );
    normalEstimator.setKSearch ( 15 );
    normalEstimator.compute ( *normals );
}

bool SICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;
    AddNormal(input_target_,target_normals_);
    return true;
}

bool SICPRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    LOG(INFO)<<"scan match";
    input_source_ = input_source;

    // pre-process input source:
    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    Eigen::Matrix3Xd X(3,transformed_input_source->size());
    Eigen::Matrix3Xd Y(3,input_target_->size());
//    Eigen::Matrix3Xd N(3,target_normals_->size());

    LOG(INFO)<<"x size : "<<X.size()<<" y size : "<<Y.size();
    for(size_t i = 0;i < transformed_input_source->size();i++){
        X(0,i) = transformed_input_source->points[i].x;
        X(1,i) = transformed_input_source->points[i].y;
        X(2,i) = transformed_input_source->points[i].z;
    }

    for(size_t i = 0; i < input_target_->size(); i++)
    {
        Y(0,i) = input_target_->points[i].x;
        Y(1,i) = input_target_->points[i].y;
        Y(2,i) = input_target_->points[i].z;
    }

    Eigen::Affine3d result = SICP::point_to_point(X,Y,params_);

    transformation_.setIdentity();
    transformation_ = result.cast<float>().matrix();
    // set output:
    result_pose = transformation_ * predict_pose;

//    Eigen::Quaternionf  quaternionf(result_pose.block<3,3>(0,0));
//    quaternionf.normalize();
//    Eigen::Vector3f  t  = result_pose.block<3,1>(0,3);
//    result_pose.setIdentity();
//    result_pose.block<3,3>(0,0) = quaternionf.toRotationMatrix();
//    result_pose.block<3,1>(0,3) = t;

    LOG(INFO)<<"result pose : "<<result_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

} // namespace lidar_localization