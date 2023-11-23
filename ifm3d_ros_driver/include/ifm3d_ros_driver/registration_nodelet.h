#ifndef __IFM3D_ROS_REGISTRATION_NODELET_H__
#define __IFM3D_ROS_REGISTRATION_NODELET_H__

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ifm3d_ros_msgs/Intrinsics.h>
#include <ifm3d_ros_msgs/RGBInfo.h>

namespace ifm3d_ros
{
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::CompressedImage, sensor_msgs::PointCloud2> ApproximateSyncPolicy;
typedef message_filters::Subscriber<sensor_msgs::CompressedImage> CompressedImageSubscriber;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloud2Subscriber;
typedef message_filters::Synchronizer<ApproximateSyncPolicy> ImageCloudSynchronizer;

struct Intrinsics
{
  union
  {
    struct
    {
        float fx;
        float fy;
        float mx;
        float my;
        float alpha;
        float k1;
        float k2;
        float k3;
        float k4;
        float k5;
    };
    float parameter_buffer[10] = { 0.0 };
  };
  int model_id{-1};

  Intrinsics()
  {}

  Intrinsics(const std::vector<float>& parameter_buffer, int model_id)
  {
    std::copy(
      parameter_buffer.begin(),
      parameter_buffer.begin() + 10,
      std::begin(this->parameter_buffer)
    );
    this->model_id = model_id;
  }
};

class RegistrationNodelet : public nodelet::Nodelet
{
private:
  void onInit() override;
  void rgbInfoCB(ifm3d_ros_msgs::RGBInfoConstPtr rgb_info);
  void syncCB(sensor_msgs::CompressedImageConstPtr rgb_image, sensor_msgs::PointCloud2ConstPtr xyz_cloud);
  cv::Vec2i projectPoint(const tf2::Vector3& point, const Intrinsics& intrinsics);

  ros::NodeHandle nh_;
  std::unique_ptr<image_transport::ImageTransport> it_;
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

  Intrinsics rgb_info_;

  ros::Subscriber rgb_info_sub_;
  std::unique_ptr<CompressedImageSubscriber> rgb_image_sub_;
  std::unique_ptr<PointCloud2Subscriber> xyz_cloud_sub_;
  std::unique_ptr<ImageCloudSynchronizer> synchronizer_;

  ros::Publisher xyzrgb_cloud_pub_;
};  // end: class RegistrationNodelet
}  // namespace ifm3d_ros
#endif  // __IFM3D_ROS_REGISTRATION_NODELET_H__
