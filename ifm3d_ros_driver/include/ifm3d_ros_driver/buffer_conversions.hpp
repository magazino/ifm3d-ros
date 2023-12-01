// -*- c++ -*-
/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2019 ifm electronic, gmbh
 */

#ifndef IFM3D_ROS_BUFFER_CONVERSIONS_HPP_
#define IFM3D_ROS_BUFFER_CONVERSIONS_HPP_

#include <ifm3d/deserialize.h>
#include <ifm3d/fg.h>
#include <ifm3d_ros_msgs/Extrinsics.h>
#include <ifm3d_ros_msgs/Intrinsics.h>
#include <ifm3d_ros_msgs/InverseIntrinsics.h>
#include <ifm3d_ros_msgs/RGBInfo.h>
#include <ifm3d_ros_msgs/TOFInfo.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>

namespace ifm3d_ros
{
sensor_msgs::Image ifm3d_to_ros_image(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                             // image.end() don't have const overloads.
                                      const std_msgs::Header& header, const std::string& logger)
{
  static constexpr auto max_pixel_format = static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32F3);
  static auto image_format_info = [] {
    auto image_format_info = std::array<const char*, max_pixel_format + 1>{};

    {
      using namespace ifm3d;
      using namespace sensor_msgs::image_encodings;
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8U)] = TYPE_8UC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_8S)] = TYPE_8SC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U)] = TYPE_16UC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16S)] = TYPE_16SC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32U)] = "32UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32S)] = TYPE_32SC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F)] = TYPE_32FC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64U)] = "64UC1";
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_64F)] = TYPE_64FC1.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_16U2)] = TYPE_16UC2.c_str();
      image_format_info[static_cast<std::size_t>(pixel_format::FORMAT_32F3)] = TYPE_32FC3.c_str();
    }

    return image_format_info;
  }();

  const auto format = static_cast<std::size_t>(image.dataFormat());

  sensor_msgs::Image result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = 0;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (format >= max_pixel_format)
  {
    ROS_ERROR_NAMED(logger, "Pixel format out of range (%ld >= %ld)", format, max_pixel_format);
    return result;
  }

  result.encoding = image_format_info.at(format);
  result.step = result.width * sensor_msgs::image_encodings::bitDepth(image_format_info.at(format)) / 8;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.step * result.height));

  if (result.encoding.empty())
  {
    ROS_WARN_NAMED(logger, "Can't handle encoding %ld (32U == %ld, 64U == %ld)", format,
                   static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_32U),
                   static_cast<std::size_t>(ifm3d::pixel_format::FORMAT_64U));
    result.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  }

  return result;
}

sensor_msgs::Image ifm3d_to_ros_image(ifm3d::Buffer&& image, const std_msgs::Header& header,
                                           const std::string& logger)
{
  return ifm3d_to_ros_image(image, header, logger);
}

sensor_msgs::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer& image,  // Need non-const image because
                                                                                       // image.begin(), image.end()
                                                                                       // don't have const overloads.
                                                                const std_msgs::Header& header,
                                                                const std::string& format,  // "jpeg" or "png"
                                                                const std::string& logger)
{
  sensor_msgs::CompressedImage result{};
  result.header = header;
  result.format = format;

  if (const auto dataFormat = image.dataFormat();
      dataFormat != ifm3d::pixel_format::FORMAT_8S && dataFormat != ifm3d::pixel_format::FORMAT_8U)
  {
    ROS_ERROR_NAMED(logger, "Invalid data format for %s data (%ld)", format.c_str(), static_cast<std::size_t>(dataFormat));
    return result;
  }

  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), image.width() * image.height()));
  return result;
}

sensor_msgs::CompressedImage ifm3d_to_ros_compressed_image(ifm3d::Buffer&& image,
                                                                const std_msgs::Header& header,
                                                                const std::string& format, const std::string& logger)
{
  return ifm3d_to_ros_compressed_image(image, header, format, logger);
}

sensor_msgs::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer& image,  // Need non-const image because image.begin(),
                                                                        // image.end() don't have const overloads.
                                                 const std_msgs::Header& header, const std::string& logger)
{
  sensor_msgs::PointCloud2 result{};
  result.header = header;
  result.height = image.height();
  result.width = image.width();
  result.is_bigendian = false;

  if (image.begin<std::uint8_t>() == image.end<std::uint8_t>())
  {
    return result;
  }

  if (image.dataFormat() != ifm3d::pixel_format::FORMAT_32F3 && image.dataFormat() != ifm3d::pixel_format::FORMAT_32F)
  {
    ROS_ERROR_NAMED(logger, "Unsupported pixel format %ld for point cloud", static_cast<std::size_t>(image.dataFormat()));
    return result;
  }

  sensor_msgs::PointField x_field{};
  x_field.name = "x";
  x_field.offset = 0;
  x_field.datatype = sensor_msgs::PointField::FLOAT32;
  x_field.count = 1;

  sensor_msgs::PointField y_field{};
  y_field.name = "y";
  y_field.offset = 4;
  y_field.datatype = sensor_msgs::PointField::FLOAT32;
  y_field.count = 1;

  sensor_msgs::PointField z_field{};
  z_field.name = "z";
  z_field.offset = 8;
  z_field.datatype = sensor_msgs::PointField::FLOAT32;
  z_field.count = 1;

  result.fields = {
    x_field,
    y_field,
    z_field,
  };

  result.point_step = result.fields.size() * sizeof(float);
  result.row_step = result.point_step * result.width;
  result.is_dense = true;
  result.data.insert(result.data.end(), image.ptr<>(0), std::next(image.ptr<>(0), result.row_step * result.height));

  return result;
}

sensor_msgs::PointCloud2 ifm3d_to_ros_cloud(ifm3d::Buffer&& image, const std_msgs::Header& header,
                                                 const std::string& logger)
{
  return ifm3d_to_ros_cloud(image, header, logger);
}

ifm3d_ros_msgs::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer& buffer, const std_msgs::Header& header,
                                                const std::string& logger)
{
  ifm3d_ros_msgs::Extrinsics extrinsics_msg;
  extrinsics_msg.header = header;
  try
  {
    extrinsics_msg.tx = buffer.at<double>(0);
    extrinsics_msg.ty = buffer.at<double>(1);
    extrinsics_msg.tz = buffer.at<double>(2);
    extrinsics_msg.rot_x = buffer.at<double>(3);
    extrinsics_msg.rot_y = buffer.at<double>(4);
    extrinsics_msg.rot_z = buffer.at<double>(5);
  }
  catch (const std::out_of_range& ex)
  {
    ROS_WARN_NAMED(logger, "Out-of-range error fetching extrinsics");
  }
  return extrinsics_msg;
}

ifm3d_ros_msgs::Extrinsics ifm3d_to_extrinsics(ifm3d::Buffer&& buffer, const std_msgs::Header& header,
                                                const std::string& logger)
{
  return ifm3d_to_extrinsics(buffer, header, logger);
}

sensor_msgs::CameraInfo ifm3d_to_camera_info(ifm3d::Buffer& buffer, const std_msgs::Header& header,
                                                  const uint32_t height, const uint32_t width,
                                                  const std::string& logger)
{
  ifm3d::calibration::IntrinsicCalibration intrinsic;
  intrinsic.Read(buffer.ptr<uint8_t>(0));

  sensor_msgs::CameraInfo camera_info_msg;
  camera_info_msg.header = header;

  try
  {
    camera_info_msg.height = height;
    camera_info_msg.width = width;
    camera_info_msg.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

    // Read data from buffer
    const float fx = intrinsic.model_parameters[0];
    const float fy = intrinsic.model_parameters[1];
    const float mx = intrinsic.model_parameters[2];
    const float my = intrinsic.model_parameters[3];
    const float alpha = intrinsic.model_parameters[4];
    const float k1 = intrinsic.model_parameters[5];
    const float k2 = intrinsic.model_parameters[6];
    const float k3 = intrinsic.model_parameters[7];
    const float k4 = intrinsic.model_parameters[8];
    // next in buffer is k5 for bouguet or theta_max for fisheye model, both not needed here

    const float ix = width - 1;
    const float iy = height - 1;
    const float cy = (iy + 0.5 - my) / fy;
    const float cx = (ix + 0.5 - mx) / fx - alpha * cy;
    const float r2 = cx * cx + cy * cy;
    const float h = 2 * cx * cy;
    const float tx = k3 * h + k4 * (r2 + 2 * cx * cx);
    const float ty = k3 * (r2 + 2 * cy * cy) + k4 * h;

    // Distortion parameters
    camera_info_msg.D.resize(5);
    camera_info_msg.D[0] = k1;
    camera_info_msg.D[1] = k2;
    camera_info_msg.D[2] = tx;  // TODO t1 == tx ?
    camera_info_msg.D[3] = ty;  // TODO t2 == ty ?
    camera_info_msg.D[4] = k3;

    // Intrinsic camera matrix
    camera_info_msg.K[0] = fx;
    camera_info_msg.K[4] = fy;
    camera_info_msg.K[2] = cx;
    camera_info_msg.K[5] = cy;
    camera_info_msg.K[8] = 1.0;  // fixed to 1.0

    // Projection matrix
    camera_info_msg.P[0] = fx;
    camera_info_msg.P[5] = fy;
    camera_info_msg.P[2] = cx;
    camera_info_msg.P[6] = cy;
    camera_info_msg.P[10] = 1.0;  // fixed to 1.0

    ROS_DEBUG_ONCE_NAMED(logger,
                         "Intrinsics:\nfx=%f \nfy=%f \nmx=%f \nmy=%f \nalpha=%f \nk1=%f \nk2=%f \nk3=%f \nk4=%f "
                         "\nCalculated:\nix=%f \niy=%f \ncx=%f \ncy=%f \nr2=%f \nh=%f \ntx=%f \nty=%f",
                         fx, fy, mx, my, alpha, k1, k2, k3, k4, ix, iy, cx, cy, r2, h, tx, ty);
  }
  catch (const std::out_of_range& ex)
  {
    ROS_WARN_NAMED(logger, "Out-of-range error fetching intrinsics");
  }

  return camera_info_msg;
}

sensor_msgs::CameraInfo ifm3d_to_camera_info(ifm3d::Buffer&& buffer, const std_msgs::Header& header,
                                                  const uint32_t height, const uint32_t width,
                                                  const std::string& logger)

{
  return ifm3d_to_camera_info(buffer, header, height, width, logger);
}

ifm3d_ros_msgs::Intrinsics ifm3d_to_intrinsics(ifm3d::Buffer& buffer, const std_msgs::Header& header,
                                                const std::string& logger)
{
  ifm3d_ros_msgs::Intrinsics intrinsics_msg;
  intrinsics_msg.header = header;

  ifm3d::calibration::IntrinsicCalibration intrinsics;

  try
  {
    intrinsics.Read(buffer.ptr<uint8_t>(0));
    intrinsics_msg.model_id = intrinsics.model_id;
    intrinsics_msg.model_parameters = std::vector<float>(
      intrinsics.model_parameters.begin(), intrinsics.model_parameters.end()
    );
  }
  catch (...)
  {
    ROS_ERROR_NAMED(logger, "Failed to read intrinsics.");
  }

  return intrinsics_msg;
}

ifm3d_ros_msgs::Intrinsics ifm3d_to_intrinsics(ifm3d::Buffer&& buffer, const std_msgs::Header& header,
                                                const std::string& logger)

{
  return ifm3d_to_intrinsics(buffer, header, logger);
}

ifm3d_ros_msgs::InverseIntrinsics ifm3d_to_inverse_intrinsics(ifm3d::Buffer& buffer,
                                                               const std_msgs::Header& header,
                                                               const std::string& logger)
{
  ifm3d_ros_msgs::InverseIntrinsics inverse_intrinsics_msg;
  inverse_intrinsics_msg.header = header;

  ifm3d::calibration::InverseIntrinsicCalibration inverse_intrinsics;

  try
  {
    inverse_intrinsics.Read(buffer.ptr<uint8_t>(0));
    inverse_intrinsics_msg.model_id = inverse_intrinsics.model_id;
    inverse_intrinsics_msg.model_parameters = std::vector<float>(
      inverse_intrinsics.model_parameters.begin(),
      inverse_intrinsics.model_parameters.end()
    );
  }
  catch (...)
  {
    ROS_ERROR_NAMED(logger, "Failed to read inverse intrinsics.");
  }

  return inverse_intrinsics_msg;
}

ifm3d_ros_msgs::InverseIntrinsics ifm3d_to_inverse_intrinsics(ifm3d::Buffer&& buffer,
                                                               const std_msgs::Header& header,
                                                               const std::string& logger)

{
  return ifm3d_to_inverse_intrinsics(buffer, header, logger);
}

ifm3d_ros_msgs::RGBInfo ifm3d_to_rgb_info(ifm3d::Buffer& buffer, const std_msgs::Header& header,
                                           const std::string& logger)
{
  ifm3d_ros_msgs::RGBInfo rgb_info_msg;
  rgb_info_msg.header = header;

  try
  {
    auto rgb_info = ifm3d::RGBInfoV1::Deserialize(buffer);

    rgb_info_msg.version = rgb_info.version;
    rgb_info_msg.frame_counter = rgb_info.frame_counter;
    rgb_info_msg.timestamp_ns = rgb_info.timestamp_ns;
    rgb_info_msg.exposure_time = rgb_info.exposure_time;

    rgb_info_msg.extrinsics.header = header;
    rgb_info_msg.extrinsics.tx = rgb_info.extrinsic_optic_to_user.trans_x;
    rgb_info_msg.extrinsics.ty = rgb_info.extrinsic_optic_to_user.trans_y;
    rgb_info_msg.extrinsics.tz = rgb_info.extrinsic_optic_to_user.trans_z;
    rgb_info_msg.extrinsics.rot_x = rgb_info.extrinsic_optic_to_user.rot_x;
    rgb_info_msg.extrinsics.rot_y = rgb_info.extrinsic_optic_to_user.rot_y;
    rgb_info_msg.extrinsics.rot_z = rgb_info.extrinsic_optic_to_user.rot_z;

    rgb_info_msg.intrinsics.header = header;
    rgb_info_msg.intrinsics.model_id = rgb_info.intrinsic_calibration.model_id;
    rgb_info_msg.intrinsics.model_parameters = std::vector<float>(
      rgb_info.intrinsic_calibration.model_parameters.begin(),
      rgb_info.intrinsic_calibration.model_parameters.end()
    );

    rgb_info_msg.inverse_intrinsics.header = header;
    rgb_info_msg.inverse_intrinsics.model_id = rgb_info.inverse_intrinsic_calibration.model_id;
    rgb_info_msg.inverse_intrinsics.model_parameters = std::vector<float>(
      rgb_info.inverse_intrinsic_calibration.model_parameters.begin(),
      rgb_info.inverse_intrinsic_calibration.model_parameters.end()
    );
  }
  catch (...)
  {
    ROS_ERROR_NAMED(logger, "Failed to read rgb info.");
  }

  return rgb_info_msg;
}

ifm3d_ros_msgs::RGBInfo ifm3d_to_rgb_info(ifm3d::Buffer&& buffer, const std_msgs::Header& header,
                                           const std::string& logger)

{
  return ifm3d_to_rgb_info(buffer, header, logger);
}

ifm3d_ros_msgs::TOFInfo ifm3d_to_tof_info(ifm3d::Buffer& buffer, const std_msgs::Header& header,
                                           const std::string& logger)
{
  ifm3d_ros_msgs::TOFInfo tof_info_msg;
  tof_info_msg.header = header;

  try
  {
    auto tof_info = ifm3d::TOFInfoV4::Deserialize(buffer);
    tof_info_msg.measurement_block_index = tof_info.measurement_block_index;
    tof_info_msg.measurement_range_min = tof_info.measurement_range_min;
    tof_info_msg.measurement_range_max = tof_info.measurement_range_max;
    tof_info_msg.version = tof_info.version;
    tof_info_msg.distance_resolution = tof_info.distance_resolution;
    tof_info_msg.amplitude_resolution = tof_info.amplitude_resolution;
    tof_info_msg.amp_normalization_factors = std::vector<float>(
      tof_info.amp_normalization_factors.begin(), tof_info.amp_normalization_factors.end()
    );
    tof_info_msg.exposure_timestamps_ns = std::vector<long unsigned int>(
      tof_info.exposure_timestamps_ns.begin(), tof_info.exposure_timestamps_ns.end()
    );
    tof_info_msg.exposure_times_s = std::vector<unsigned int>(
      tof_info.exposure_times_s.begin(), tof_info.exposure_times_s.end()
    );
    tof_info_msg.illu_temperature = tof_info.illu_temperature;
    tof_info_msg.mode = std::string(std::begin(tof_info.mode), std::end(tof_info.mode));
    tof_info_msg.imager = std::string(std::begin(tof_info.imager), std::end(tof_info.imager));

    tof_info_msg.extrinsics.header = header;
    tof_info_msg.extrinsics.tx = tof_info.extrinsic_optic_to_user.trans_x;
    tof_info_msg.extrinsics.ty = tof_info.extrinsic_optic_to_user.trans_y;
    tof_info_msg.extrinsics.tz = tof_info.extrinsic_optic_to_user.trans_z;
    tof_info_msg.extrinsics.rot_x = tof_info.extrinsic_optic_to_user.rot_x;
    tof_info_msg.extrinsics.rot_y = tof_info.extrinsic_optic_to_user.rot_y;
    tof_info_msg.extrinsics.rot_z = tof_info.extrinsic_optic_to_user.rot_z;

    tof_info_msg.intrinsics.header = header;
    tof_info_msg.intrinsics.model_id = tof_info.intrinsic_calibration.model_id;
    tof_info_msg.intrinsics.model_parameters = std::vector<float>(
      tof_info.intrinsic_calibration.model_parameters.begin(),
      tof_info.intrinsic_calibration.model_parameters.end()
    );

    tof_info_msg.inverse_intrinsics.header = header;
    tof_info_msg.inverse_intrinsics.model_id = tof_info.inverse_intrinsic_calibration.model_id;
    tof_info_msg.inverse_intrinsics.model_parameters = std::vector<float>(
      tof_info.inverse_intrinsic_calibration.model_parameters.begin(),
      tof_info.inverse_intrinsic_calibration.model_parameters.end()
    );
  }
  catch (...)
  {
    ROS_ERROR_NAMED(logger, "Failed to read tof info.");
  }

  return tof_info_msg;
}

ifm3d_ros_msgs::TOFInfo ifm3d_to_tof_info(ifm3d::Buffer&& buffer, const std_msgs::Header& header,
                                           const std::string& logger)

{
  return ifm3d_to_tof_info(buffer, header, logger);
}

}  // namespace ifm3d_ros

#endif