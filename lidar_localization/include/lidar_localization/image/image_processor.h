//
// Created by melody on 2022/12/10.
//

#ifndef LIDAR_LOCALIZATION_IMAGE_PROCESSOR_H
#define LIDAR_LOCALIZATION_IMAGE_PROCESSOR_H
#include <iostream>
#include <map>
#include <vector>

#include <glog/logging.h>

#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "lidar_localization/image/image_utils.h"

namespace lidar_localization {
class ImageProcessor {
  public:
    ImageProcessor(ros::NodeHandle &nh);
    ~ImageProcessor();
    typedef std::shared_ptr<ImageProcessor> Ptr;
    typedef std::shared_ptr<const ImageProcessor> ConstPtr;

  private:
    /*
     * 对比两个特征点的response
     */
    static bool KeyPointCompareByResponse(const cv::KeyPoint &pt1,
                                          const cv::KeyPoint &pt2) {
        return pt1.response > pt2.response;
    }

    /*
     * 对别两个特征的response
     */
    static bool FeatureCompareByResponse(const FeatureMetaData &f1,
                                         const FeatureMetaData &f2) {
        return f1.response > f2.response;
    }

    static bool FeatureCompareByLifetime(const FeatureMetaData &f1,
                                         const FeatureMetaData &f2) {
        return f1.lifetime > f2.lifetime;
    }
    /*
     * create ros publisher and subscriber
     */
    bool CreateRosIO();

    void StereoCallback(const sensor_msgs::ImageConstPtr &cam0_img,
                        const sensor_msgs::ImageConstPtr &cam1_img);

    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);

  private:
    bool is_first_img_;
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    message_filters::Subscriber<sensor_msgs::Image> cam0_img_sub_;
    message_filters::Subscriber<sensor_msgs::Image> cam1_img_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
        stereo_sub_;
};

typedef ImageProcessor::Ptr ImageProcessorPtr;
typedef ImageProcessor::ConstPtr ImageProcessorConstPtr;

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_IMAGE_PROCESSOR_H
