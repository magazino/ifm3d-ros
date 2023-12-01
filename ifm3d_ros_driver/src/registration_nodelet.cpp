#include <ifm3d_ros_driver/registration_nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace ifm3d_ros
{
void RegistrationNodelet::onInit()
{
  nh_ = getMTPrivateNodeHandle();
  it_ = std::make_unique<image_transport::ImageTransport>(nh_);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(5.0));
  tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_buffer_);

  std::string rgb_image_topic;
  std::string rgb_info_topic;
  std::string xyz_cloud_topic;
  std::string xyzrgb_cloud_topic;

  nh_.getParam("rgb_image_topic", rgb_image_topic);
  nh_.getParam("rgb_info_topic", rgb_info_topic);
  nh_.getParam("xyz_cloud_topic", xyz_cloud_topic);
  nh_.getParam("xyzrgb_cloud_topic", xyzrgb_cloud_topic);

  xyz_cloud_sub_ = std::make_unique<PointCloud2Subscriber>(nh_, xyz_cloud_topic, 1);
  rgb_image_sub_ = std::make_unique<CompressedImageSubscriber>(nh_, rgb_image_topic, 1);
  synchronizer_ = std::make_unique<ImageCloudSynchronizer>(
    ApproximateSyncPolicy(1), *rgb_image_sub_.get(), *xyz_cloud_sub_.get()
  );
  synchronizer_->registerCallback(boost::bind(&RegistrationNodelet::syncCB, this, _1, _2));

  rgb_info_sub_ = nh_.subscribe(rgb_info_topic, 1, &RegistrationNodelet::rgbInfoCB, this);
  xyzrgb_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(xyzrgb_cloud_topic, 1, false);
}

void RegistrationNodelet::rgbInfoCB(ifm3d_ros_msgs::RGBInfoConstPtr rgb_info)
{
  rgb_info_ = Intrinsics(rgb_info->inverse_intrinsics.model_parameters, rgb_info->inverse_intrinsics.model_id);
  rgb_info_sub_ = ros::Subscriber();  // unsubscribe
}

cv::Vec2i RegistrationNodelet::projectPoint(const tf2::Vector3& point, const Intrinsics& intrinsics)
{
  cv::Vec2i pixel;
  if (rgb_info_.model_id == 0 || rgb_info_.model_id == 1)
  {
    // Bouguet model
    auto tz = std::max(0.001, point.getZ());
    auto ixn = point.getX() / tz;
    auto iyn = point.getY() / tz;
    auto rd2 = ixn * ixn + iyn * iyn;
    auto radial = rd2 * (rgb_info_.k1 + rd2 * (rgb_info_.k2 + rd2 * rgb_info_.k5)) + 1;
    auto ixd = ixn * radial;
    auto iyd = iyn * radial;
    if (rgb_info_.k3 != 0 || rgb_info_.k4 != 0)
    {
      auto h = 2 * ixn * iyn;
      auto tangx = rgb_info_.k3 * h + rgb_info_.k4 * (rd2 + 2 * ixn * ixn);
      auto tangy = rgb_info_.k3 * (rd2 + 2 * iyn * iyn) + rgb_info_.k4 * h;
      ixd += tangx;
      iyd += tangy;
    }
    pixel[0] = static_cast<int>(std::round(((rgb_info_.fx * (ixd + rgb_info_.alpha * iyd)) + rgb_info_.mx) - 0.5));
    pixel[1] = static_cast<int>(std::round(((rgb_info_.fy * (iyd)) + rgb_info_.my) - 0.5));
  }
  else if (rgb_info_.model_id == 2 || rgb_info_.model_id == 3)
  {
    // fisheye model
    auto lxy = std::sqrt(point.getX() * point.getX() + point.getY() * point.getY());
    auto theta = std::atan2(lxy, point.getZ());
    auto phi = std::min(theta, double(rgb_info_.k5));
    phi *= phi;
    auto p_radial = 1 + phi * (rgb_info_.k1 + phi * (rgb_info_.k2 + phi * (rgb_info_.k3 + phi * rgb_info_.k4)));
    auto theta_s = p_radial * theta;
    auto f_radial = (lxy > 0.0) ? (theta_s / lxy) : 0.0;
    auto ixd = f_radial * point.getX();
    auto iyd = f_radial * point.getY();
    pixel[0] = static_cast<int>(std::round(
      ixd * rgb_info_.fx - 0.5 + rgb_info_.mx - rgb_info_.alpha * iyd * rgb_info_.fx
    ));
    pixel[1] = static_cast<int>(std::round(iyd * rgb_info_.fy - 0.5 + rgb_info_.my));
  }
  return pixel;
}

void RegistrationNodelet::syncCB(
  sensor_msgs::CompressedImageConstPtr rgb_image_msg,
  sensor_msgs::PointCloud2ConstPtr xyz_cloud)
{
  if (!rgb_info_sub_.getTopic().empty())
  {
    // RGB info not yet received
    return;
  }

  sensor_msgs::PointCloud2 xyzrgb_cloud;
  sensor_msgs::PointCloud2Modifier modifier(xyzrgb_cloud);

  modifier.setPointCloud2Fields(6,
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32,
    "r", 1, sensor_msgs::PointField::FLOAT32,
    "g", 1, sensor_msgs::PointField::FLOAT32,
    "b", 1, sensor_msgs::PointField::FLOAT32
  );

  xyzrgb_cloud.header = std_msgs::Header();
  xyzrgb_cloud.header.stamp = xyz_cloud->header.stamp;
  xyzrgb_cloud.header.frame_id = xyz_cloud->header.frame_id;
  xyzrgb_cloud.height = xyz_cloud->height;
  xyzrgb_cloud.width = xyz_cloud->width;
  xyzrgb_cloud.is_dense = true;
  xyzrgb_cloud.point_step = 6 * sizeof(float);
  xyzrgb_cloud.row_step = xyzrgb_cloud.point_step * xyzrgb_cloud.height * xyzrgb_cloud.width;
  xyzrgb_cloud.data.resize(xyzrgb_cloud.row_step);

  geometry_msgs::TransformStamped rgb_to_cloud_tf_msg;
  tf2::Transform rgb_to_cloud_tf;
  try
  {
    rgb_to_cloud_tf_msg = tf2_buffer_->lookupTransform(
      rgb_image_msg->header.frame_id, xyz_cloud->header.frame_id,
      xyz_cloud->header.stamp, ros::Duration(0.3)
    );
  }
  catch (const tf2::TransformException& e)
  {
    NODELET_WARN_THROTTLE(5, "TF Exception: %s", e.what());
    return;
  }
  tf2::fromMsg(rgb_to_cloud_tf_msg.transform, rgb_to_cloud_tf);

  cv_bridge::CvImageConstPtr rgb_image = cv_bridge::toCvCopy(rgb_image_msg, sensor_msgs::image_encodings::RGB8);

  sensor_msgs::PointCloud2ConstIterator<float> it_xyz(*xyz_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_xyzrgb(xyzrgb_cloud, "x");
  for (; it_xyz != it_xyz.end(); ++it_xyz, ++it_xyzrgb)
  {
    tf2::Vector3 point_in_rgb = rgb_to_cloud_tf * tf2::Vector3(it_xyz[0], it_xyz[1], it_xyz[2]);
    cv::Vec2i pixel_in_rgb = projectPoint(point_in_rgb, rgb_info_);
    if (pixel_in_rgb[0] >= rgb_image->image.size[1] || pixel_in_rgb[1] >= rgb_image->image.size[0] ||
        pixel_in_rgb[0] < 0 || pixel_in_rgb[1] < 0)
    {
      continue;
    }
    cv::Vec3b pixel_color = rgb_image->image.at<cv::Vec3b>(pixel_in_rgb[1], pixel_in_rgb[0]);
    it_xyzrgb[0] = it_xyz[0];
    it_xyzrgb[1] = it_xyz[1];
    it_xyzrgb[2] = it_xyz[2];
    it_xyzrgb[3] = static_cast<float>(pixel_color[0]) / 255.;
    it_xyzrgb[4] = static_cast<float>(pixel_color[1]) / 255.;
    it_xyzrgb[5] = static_cast<float>(pixel_color[2]) / 255.;
  }

  xyzrgb_cloud_pub_.publish(xyzrgb_cloud);
}
}  // namespace ifm3d_ros

PLUGINLIB_EXPORT_CLASS(ifm3d_ros::RegistrationNodelet, nodelet::Nodelet)
