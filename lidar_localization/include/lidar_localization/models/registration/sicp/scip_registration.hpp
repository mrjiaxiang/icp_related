/*
 * @Description: SICP registration 
 * @Author: Ge Yao
 * @Date: 2021-04-24 08:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_

#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/registration/sicp/ICP.h"

using namespace SICP;

namespace lidar_localization {

class SICPRegistration: public RegistrationInterface {
  public:
    SICPRegistration(const Json::Value& node);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(
      const CloudData::CLOUD_PTR& input_source, 
      const Eigen::Matrix4f& predict_pose, 
      CloudData::CLOUD_PTR& result_cloud_ptr,
      Eigen::Matrix4f& result_pose
    ) override;
    bool AddNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   pcl::PointCloud<pcl::Normal>::Ptr normals);

  private:
    CloudData::CLOUD_PTR input_target_;
    CloudData::CLOUD_PTR input_source_;
    pcl::PointCloud<pcl::Normal>::Ptr target_normals_;
    Parameters params_;
    Eigen::Matrix4f transformation_;

};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_MODELS_REGISTRATION_SICP_REGISTRATION_HPP_