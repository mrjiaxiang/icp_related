//
// Created by melody on 2022/12/10.
//

#include "lidar_localization/image/image_processor.h"

namespace lidar_localization {
ImageProcessor::ImageProcessor(ros::NodeHandle &nh)
    : nh_(nh), is_first_img_(true), stereo_sub_(10) {}
ImageProcessor::~ImageProcessor() {}

bool ImageProcessor::CreateRosIO() {
    cam0_img_sub_.subscribe(nh_, "cam0_image", 10);
    cam1_img_sub_.subscribe(nh_, "cam1_image", 10);

    stereo_sub_.connectInput(cam0_img_sub_, cam1_img_sub_);
    stereo_sub_.registerCallback(&ImageProcessor::StereoCallback, this);
    imu_sub_ = nh_.subscribe("imu", 50, &ImageProcessor::ImuCallback, this);
    return true;
}

void ImageProcessor::StereoCallback(
    const sensor_msgs::ImageConstPtr &cam0_img,
    const sensor_msgs::ImageConstPtr &cam1_img) {

    ROS_INFO("stereo callback.");
    return;
}

void ImageProcessor::ImuCallback(const sensor_msgs::ImuConstPtr &msg) {
    ROS_INFO("IMU call back.");
    return;
}

} // namespace lidar_localization