//
// Created by melody on 2022/12/10.
//

#ifndef LIDAR_LOCALIZATION_IMAGE_UTILS_H
#define LIDAR_LOCALIZATION_IMAGE_UTILS_H

#include <ros/ros.h>

#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>

#include <Eigen/Geometry>

#include <glog/logging.h>

namespace lidar_localization {
struct ProcessorConfig {
    int grid_row;
    int grid_col;
    int grid_min_feature_num;
    int grid_max_feature_num;

    int pyramid_levels;
    int patch_size;
    int fast_threshold;
    int max_iteration;
    double track_precision;
    double ransac_threshold;
    double stereo_threshold;
};

typedef unsigned long long int FeatureIDType;

struct FeatureMetaData {
    FeatureIDType id;
    float response;
    int lifetime; // 每次被追踪一次,lifetime就+1
    cv::Point2f cam0_point;
    cv::Point2f cam1_point;
};

typedef std::map<int, std::vector<FeatureMetaData>> GridFeatures;

inline cv::Mat getKalibrStyleTransform(const ros::NodeHandle &nh,
                                       const std::string &field) {
    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
    XmlRpc::XmlRpcValue lines;
    if (!nh.getParam(field, lines)) {
        throw(std::runtime_error("cannot find transform " + field));
    }
    if (lines.size() != 4 ||
        lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        throw(std::runtime_error("invalid transform " + field));
    }
    for (int i = 0; i < lines.size(); i++) {
        if (lines.size() != 4 ||
            lines.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw(std::runtime_error("bad line for transform " + field));
        }
        for (int j = 0; j < lines[i].size(); j++) {
            if (lines[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble) {
                throw(std::runtime_error("bad value for transform " + field));
            } else {
                T.at<double>(i, j) = static_cast<double>(lines[i][j]);
            }
        }
    }

    return T;
}

// inline cv::Mat getTransformCV(const ros::NodeHandle &nh,
//                        const std::string &field){
//     cv::Mat T;
//     try {
//         T =getKalibrStyleTransform(nh,field);
//     }catch (std::runtime_error& e){
//         LOG(INFO)<<"cannot read transform " << field
//                  << " in kalibr format, trying old one!";
//         try {
//             T =
//         }catch (std::runtime_error& e){
//             std::string error = "cannot read transform " + field + " error: "
//             + e.what(); LOG(ERROR)<<error; throw std::runtime_error(error);
//         }
//     }

//     return T;
// }

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_IMAGE_UTILS_H
