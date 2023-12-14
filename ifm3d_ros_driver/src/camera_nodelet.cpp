/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021 ifm electronic, gmbh
 */

#include <ifm3d/fg/frame.h>
#include <ifm3d_ros_driver/buffer_conversions.hpp>
#include <ifm3d_ros_driver/camera_nodelet.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>

#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <ifm3d_ros_msgs/Config.h>
#include <ifm3d_ros_msgs/Dump.h>
#include <ifm3d_ros_msgs/Extrinsics.h>
#include <ifm3d_ros_msgs/SoftOff.h>
#include <ifm3d_ros_msgs/SoftOn.h>
#include <ifm3d_ros_msgs/Trigger.h>



using json = ifm3d::json;
using namespace std::chrono_literals;
// This function formats the timestamps for proper display
// a.k.a converts to local time
std::string formatTimestamp(ifm3d::TimePointT timestamp)
{
  using namespace std::chrono;
  std::time_t time = std::chrono::system_clock::to_time_t(
    std::chrono::time_point_cast<std::chrono::system_clock::duration>(
      timestamp));

  milliseconds milli = duration_cast<milliseconds>(
    timestamp.time_since_epoch() -
    duration_cast<seconds>(timestamp.time_since_epoch()));

  std::ostringstream s;
  s << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << ":"
    << std::setw(3) << std::setfill('0') << milli.count();

  return s.str();
}

using json = ifm3d::json;
namespace enc = sensor_msgs::image_encodings;

inline const char * const BoolToString(bool b)
{
  return b ? "true" : "false";
}

void ifm3d_ros::CameraNodelet::onInit()
{
  std::string nn = this->getName();
  NODELET_INFO_STREAM("onInit(): " << nn);

  this->np_ = getMTPrivateNodeHandle();
  this->it_.reset(new image_transport::ImageTransport(this->np_));

  //
  // parse data out of the parameter server
  //

  int xmlrpc_port;
  int pcic_port;
  std::string frame_id_base;
  bool xyz_image_stream;
  bool confidence_image_stream;
  bool radial_distance_image_stream;
  bool radial_distance_noise_stream;
  bool norm_amplitude_image_stream;
  bool amplitude_image_stream;
  bool extrinsic_image_stream;
  bool intrinsic_image_stream;
  bool rgb_image_stream;
  bool rgb_info_stream;

  if ((nn.size() > 0) && (nn.at(0) == '/'))
  {
    frame_id_base = nn.substr(1);
  }
  else
  {
    frame_id_base = nn;
  }

  this->np_.param("ip", this->camera_ip_, ifm3d::DEFAULT_IP);
  this->np_.param("xmlrpc_port", xmlrpc_port, (int)ifm3d::DEFAULT_XMLRPC_PORT);
  this->np_.param("pcic_port", pcic_port, (int)ifm3d::DEFAULT_PCIC_PORT);
  this->np_.param("password", this->password_, ifm3d::DEFAULT_PASSWORD);
  this->np_.param("timeout_millis", this->timeout_millis_, 500);
  this->np_.param("timeout_tolerance_secs", this->timeout_tolerance_secs_, 5.0);
  this->np_.param("assume_sw_triggered", this->assume_sw_triggered_, false);
  this->np_.param("soft_on_timeout_millis", this->soft_on_timeout_millis_, 500);
  this->np_.param("soft_on_timeout_tolerance_secs", this->soft_on_timeout_tolerance_secs_, 5.0);
  this->np_.param("soft_off_timeout_millis", this->soft_off_timeout_millis_, 500);
  this->np_.param("soft_off_timeout_tolerance_secs", this->soft_off_timeout_tolerance_secs_, 600.0);
  this->np_.param("frame_latency_thresh", this->frame_latency_thresh_, 60.0f);
  this->np_.param("frame_id_base", frame_id_base, frame_id_base);

  // image stream ros parameter server setting
  this->np_.param("xyz_image_stream", xyz_image_stream, true);
  this->np_.param("confidence_image_stream", confidence_image_stream, true);
  this->np_.param("radial_distance_image_stream", radial_distance_image_stream, true);
  this->np_.param("radial_distance_noise_stream", radial_distance_noise_stream, true);
  this->np_.param("amplitude_image_stream", amplitude_image_stream, true);
  this->np_.param("extrinsic_image_stream", extrinsic_image_stream, true);
  this->np_.param("intrinsic_image_stream", intrinsic_image_stream, true);
  this->np_.param("rgb_image_stream", rgb_image_stream, true);
  this->np_.param("rgb_info_stream", rgb_info_stream, true);

  // default schema masks
  std::list<ifm3d::buffer_id> buffer_list;
  const ifm3d::FrameGrabber::BufferList DEFAULT_SCHEMA_MASK_3D = {
                              ifm3d::buffer_id::XYZ,
                              ifm3d::buffer_id::CONFIDENCE_IMAGE,
                              ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE,
                              ifm3d::buffer_id::RADIAL_DISTANCE_NOISE,
                              ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE,
                              ifm3d::buffer_id::EXTRINSIC_CALIB,
                              ifm3d::buffer_id::INTRINSIC_CALIB,
                            };

  const ifm3d::FrameGrabber::BufferList DEFAULT_SCHEMA_MASK_2D = {
                              ifm3d::buffer_id::JPEG_IMAGE,
                              ifm3d::buffer_id::EXTRINSIC_CALIB,
                              ifm3d::buffer_id::INTRINSIC_CALIB,
                              ifm3d::buffer_id::RGB_INFO,
                            };

  this->xyz_image_stream_ = static_cast<bool>(xyz_image_stream);
  this->confidence_image_stream_ = static_cast<bool>(confidence_image_stream);
  this->radial_distance_image_stream_ = static_cast<bool>(radial_distance_image_stream);
  this->radial_distance_noise_stream_ = static_cast<bool>(radial_distance_noise_stream);
  this->amplitude_image_stream_ = static_cast<bool>(amplitude_image_stream);
  this->rgb_image_stream_ = static_cast<bool>(rgb_image_stream);
  this->rgb_info_stream_ = static_cast<bool>(rgb_info_stream);
  this->extrinsic_image_stream_ = static_cast<bool>(extrinsic_image_stream);
  this->intrinsic_image_stream_ = static_cast<bool>(intrinsic_image_stream);

  this->xmlrpc_port_ = static_cast<std::uint16_t>(xmlrpc_port);
  this->pcic_port_ = static_cast<std::uint16_t>(pcic_port);

  NODELET_INFO_ONCE("IP default: %s, current %s", ifm3d::DEFAULT_IP.c_str(), this->camera_ip_.c_str());
  NODELET_INFO_ONCE("PCIC port check: current %d, default %d", this->pcic_port_, ifm3d::DEFAULT_PCIC_PORT);
  NODELET_INFO_ONCE("XML-RPC port check: current %d, default %d", this->xmlrpc_port_, ifm3d::DEFAULT_XMLRPC_PORT);


  this->schema_mask_default_3d_ = DEFAULT_SCHEMA_MASK_3D;   // use DEFAULT_SCHEMA_MASK until implemented as yml file: list of strings
  this->schema_mask_default_2d_ = DEFAULT_SCHEMA_MASK_2D;   // use DEFAULT_SCHEMA_MASK until implemented as yml file: list of strings

  // lastly get the camera type based on PortsInfo
  this->np_.param("imager_type", this->imager_type_ , std::string("3D"));
  this->imager_type_ = GetCameraType(this->pcic_port_);

  NODELET_INFO_ONCE("Imager type current: %s, default %s", imager_type_.c_str(), "3D");
  NODELET_DEBUG_STREAM("setup ros node parameters finished");


  this->frame_id_ = frame_id_base + "_link";
  this->optical_frame_id_ = frame_id_base + "_optical_link";

  //-------------------
  // Published topics
  //-------------------
  if (strcmp(this->imager_type_.c_str(), "3D") == 0 && this->xyz_image_stream_)
  {
    this->cloud_pub_ = this->np_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    NODELET_DEBUG_STREAM("Point cloud publisher active");
  }

  if (strcmp(this->imager_type_.c_str(), "3D") == 0 && this->radial_distance_image_stream_)
  {
    this->distance_pub_ = this->it_->advertise("distance", 1);
    NODELET_DEBUG_STREAM("Distance image publisher active");
  }

  if (strcmp(this->imager_type_.c_str(), "3D") == 0 && this->radial_distance_noise_stream_)
  {
    this->distance_noise_pub_ = this->it_->advertise("distance_noise", 1);
    NODELET_DEBUG_STREAM("Distance noise image publisher active");
  }

  if (strcmp(this->imager_type_.c_str(), "3D") == 0 && this->amplitude_image_stream_)
  {
    this->amplitude_pub_ = this->it_->advertise("amplitude", 1);
    NODELET_DEBUG_STREAM("Amplitude image publisher active");
  }

  if (strcmp(this->imager_type_.c_str(), "3D") == 0 && this->confidence_image_stream_)
  {
    this->conf_pub_ = this->it_->advertise("confidence", 1);
    NODELET_DEBUG_STREAM("Confidence image publisher active");
  }

  if (strcmp(this->imager_type_.c_str(), "2D") == 0 && this->rgb_image_stream_)
  {
    this->rgb_image_pub_ = this->np_.advertise<sensor_msgs::CompressedImage>("rgb_image/compressed", 1);
    NODELET_DEBUG_STREAM("2D RGB image publisher active");
  }

  if (this->extrinsic_image_stream_)
  {
    this->extrinsics_pub_ = this->np_.advertise<ifm3d_ros_msgs::Extrinsics>("extrinsics", 1);
    NODELET_DEBUG_STREAM("Extrinsics parameter publisher active");
  }

  if (this->intrinsic_image_stream_)
  {
    this->intrinsics_pub_ = this->np_.advertise<ifm3d_ros_msgs::Intrinsics>("intrinsics", 1);
    NODELET_DEBUG_STREAM("Intrinsics parameter publisher active");
  }

  if (strcmp(this->imager_type_.c_str(), "2D") == 0 && this->rgb_info_stream_)
  {
    this->rgb_info_pub_ = this->np_.advertise<ifm3d_ros_msgs::RGBInfo>("rgb_info", 1);
    this->rgb_camera_info_pub_ = this->np_.advertise<sensor_msgs::CameraInfo>("rgb_camera_info", 1);
    NODELET_DEBUG_STREAM("RGB info publisher active");
  }

  NODELET_DEBUG_STREAM("after advertising the publishers");



  //---------------------
  // Advertised Services
  //---------------------
  this->dump_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::Dump::Request, ifm3d_ros_msgs::Dump::Response>(
      "Dump", std::bind(&CameraNodelet::Dump, this, std::placeholders::_1, std::placeholders::_2));

  this->dump_json_schema_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::DumpJSONSchema::Request, ifm3d_ros_msgs::DumpJSONSchema::Response>(
      "DumpJSONSchema", std::bind(&CameraNodelet::DumpJSONSchema, this, std::placeholders::_1, std::placeholders::_2));

  this->config_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::Config::Request, ifm3d_ros_msgs::Config::Response>(
      "Config", std::bind(&CameraNodelet::Config, this, std::placeholders::_1, std::placeholders::_2));

  this->trigger_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::Trigger::Request, ifm3d_ros_msgs::Trigger::Response>(
      "Trigger", std::bind(&CameraNodelet::Trigger, this, std::placeholders::_1, std::placeholders::_2));

  this->soft_off_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::SoftOff::Request, ifm3d_ros_msgs::SoftOff::Response>(
      "SoftOff", std::bind(&CameraNodelet::SoftOff, this, std::placeholders::_1, std::placeholders::_2));

  this->soft_on_srv_ = this->np_.advertiseService<ifm3d_ros_msgs::SoftOn::Request, ifm3d_ros_msgs::SoftOn::Response>(
      "SoftOn", std::bind(&CameraNodelet::SoftOn, this, std::placeholders::_1, std::placeholders::_2));

  NODELET_DEBUG_STREAM("after advertise service");
  ros::Duration(5.0).sleep();

  //----------------------------------
  // Fire off our main publishing loop
  //----------------------------------
  this->publoop_timer_ = this->np_.createTimer(
      ros::Duration(.001), [this](const ros::TimerEvent& t) { this->Run(); },
      true);  // oneshot timer
}

bool ifm3d_ros::CameraNodelet::Dump(ifm3d_ros_msgs::Dump::Request& req, ifm3d_ros_msgs::Dump::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  try
  {
    json j = this->cam_->ToJSON();
    res.config = j.dump();
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    NODELET_WARN_STREAM(ex.what());
  }
  catch (const std::exception& std_ex)
  {
    res.status = -1;
    NODELET_WARN_STREAM(std_ex.what());
  }
  catch (...)
  {
    res.status = -2;
  }

  if (res.status != 0)
  {
    NODELET_WARN_STREAM("Dump: " << res.status);
  }

  return true;
}


bool ifm3d_ros::CameraNodelet::DumpJSONSchema(ifm3d_ros_msgs::DumpJSONSchema::Request& req, ifm3d_ros_msgs::DumpJSONSchema::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  try
  {
    ifm3d::O3R::Ptr cam_O3R = std::static_pointer_cast<ifm3d::O3R>(this->cam_);
    json j = cam_O3R->GetSchema();
    res.config = j.dump();
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    NODELET_WARN_STREAM(ex.what());
  }
  catch (const std::exception& std_ex)
  {
    res.status = -1;
    NODELET_WARN_STREAM(std_ex.what());
  }
  catch (...)
  {
    res.status = -2;
  }

  if (res.status != 0)
  {
    NODELET_WARN_STREAM("Dump: " << res.status);
  }

  return true;
}


bool ifm3d_ros::CameraNodelet::Config(ifm3d_ros_msgs::Config::Request& req, ifm3d_ros_msgs::Config::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "OK";

  try
  {
    this->cam_->FromJSON(json::parse(req.json));
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    res.msg = ex.what();
  }
  catch (const std::exception& std_ex)
  {
    res.status = -1;
    res.msg = std_ex.what();
  }
  catch (...)
  {
    res.status = -2;
    res.msg = "Unknown error in `Config'";
  }

  if (res.status != 0)
  {
    NODELET_WARN_STREAM("Config: " << res.status << " - " << res.msg);
  }

  return true;
}

bool ifm3d_ros::CameraNodelet::Trigger(ifm3d_ros_msgs::Trigger::Request& req, ifm3d_ros_msgs::Trigger::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  res.msg = "Software trigger is currently not implemented";

  try
  {
    this->fg_->SWTrigger();
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
  }

  NODELET_WARN_STREAM("Triggering a camera head is currently not implemented - will follow");
  return true;
}


bool ifm3d_ros::CameraNodelet::SoftOff(ifm3d_ros_msgs::SoftOff::Request& req, ifm3d_ros_msgs::SoftOff::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;

  int port_arg = -1;

  try
  {
    port_arg = static_cast<int>(this->pcic_port_) % 50010;

    // Configure the device from a json string
    this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"CONF\"}}}");

    this->assume_sw_triggered_ = false;
    this->timeout_millis_ = this->soft_on_timeout_millis_;
    this->timeout_tolerance_secs_ = this->soft_on_timeout_tolerance_secs_;
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    res.msg = ex.what();
    return false;
  }

  NODELET_INFO_STREAM("Switched state to CONF");
  res.msg = "{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"CONF\"}}}";

  return true;
}


bool ifm3d_ros::CameraNodelet::SoftOn(ifm3d_ros_msgs::SoftOn::Request& req, ifm3d_ros_msgs::SoftOn::Response& res)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  res.status = 0;
  int port_arg = -1;

  try
  {
    port_arg = static_cast<int>(this->pcic_port_) % 50010;

    // Configure the device from a json string
    this->cam_->FromJSONStr("{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"RUN\"}}}");

    this->assume_sw_triggered_ = false;
    this->timeout_millis_ = this->soft_on_timeout_millis_;
    this->timeout_tolerance_secs_ = this->soft_on_timeout_tolerance_secs_;
  }
  catch (const ifm3d::Error& ex)
  {
    res.status = ex.code();
    res.msg = ex.what();
    return false;
  }

  NODELET_INFO_STREAM("Switched state to RUN");
  res.msg = "{\"ports\":{\"port" + std::to_string(port_arg) + "\": {\"state\": \"RUN\"}}}";

  return true;
}

std::string ifm3d_ros::CameraNodelet::GetCameraType(std::uint16_t pcic_port)
{
  std::lock_guard<std::mutex> lock(this->mutex_);

  auto cam_O3R = std::make_shared<ifm3d::O3R>(this->camera_ip_, this->xmlrpc_port_);
  std::vector<ifm3d::PortInfo> ports_vector_ = cam_O3R->Ports();

  int port_arg = static_cast<int>(this->pcic_port_) % 50010;
  std::string port_type_ = cam_O3R->Port("port" + std::to_string(port_arg)).type;
  NODELET_INFO_ONCE("Imager type as retrieved from API device info: PCIC port %d, type %s", (int) this->pcic_port_, port_type_.c_str());

  return port_type_;
}

bool ifm3d_ros::CameraNodelet::InitStructures(std::uint16_t pcic_port)
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  bool retval = false;

  int port_arg = -1;

  try
  {
    NODELET_INFO_STREAM("Running dtors...");
    this->fg_.reset();
    this->cam_.reset();

    NODELET_INFO_STREAM("Initializing camera...");
    this->cam_ = ifm3d::Device::MakeShared(this->camera_ip_, this->xmlrpc_port_);
    ros::Duration(5.0).sleep();

    NODELET_INFO_STREAM("Initializing framegrabber...");
    this->fg_ = std::make_shared<ifm3d::FrameGrabber>(this->cam_, this->pcic_port_);
    NODELET_INFO_ONCE("Nodelet argument: %d", (int)this->pcic_port_);
    ros::Duration(5.0).sleep();

    retval = true;
  }
  catch (const ifm3d::Error& ex)
  {
    NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
    this->fg_.reset();
    this->cam_.reset();
    retval = false;
  }

  return retval;
}

void ifm3d_ros::CameraNodelet::Callback2D(ifm3d::Frame::Ptr frame){
    //
    // Pull out all the wrapped images so that we can release the "GIL"
    // while publishing
    //
    std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);
    lock.lock();
    ifm3d::Buffer extrinsics;
    ifm3d::Buffer rgb_img;
    this->last_frame_local_time_ = ros::Time::now();
    // ifm3d::Buffer rgb_img_info;


    NODELET_DEBUG_STREAM("start getting data");
    try
    {
      rgb_img = frame->GetBuffer(ifm3d::buffer_id::JPEG_IMAGE);
      // extrinsics = frame->GetBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB);
      // rgb_img_info = frame->GetBuffer(ifm3d::buffer_id::RGB_INFO);
      // this->last_frame_time_ = frame->TimeStamps()[0];

    }
    catch (const ifm3d::Error& ex)
    {
      NODELET_WARN_STREAM(ex.what());
    }
    catch (const std::exception& std_ex)
    {
      NODELET_WARN_STREAM(std_ex.what());
    }
    NODELET_DEBUG_STREAM("finished getting data");

    lock.unlock();

    //
    // Now, do the publishing
    //

    // Timestamps:
    this->head.stamp = ros::Time(
      std::chrono::duration_cast<std::chrono::duration<double,
      std::ratio<1>>>(frame->TimeStamps()[0].time_since_epoch()).count()
      );
    if ((ros::Time::now() - this->head.stamp) > ros::Duration(this->frame_latency_thresh_))
    {
      NODELET_INFO_STREAM("Camera's time and client's time are not synced");
      this->head.stamp = ros::Time::now();
    }
    // Header frame_id
    this->head.frame_id = this->frame_id_;

    if (this->rgb_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::JPEG_IMAGE))
    {
      this->rgb_image_pub_.publish(ifm3d_to_ros_compressed_image(rgb_img, head, "jpeg", getName()));
      NODELET_DEBUG_STREAM("after publishing rgb image");
    }

    if (this->extrinsic_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB))
    {
      auto buffer = frame->GetBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB);
      this->extrinsics_pub_.publish(ifm3d_to_extrinsics(buffer, head, getName()));
      NODELET_DEBUG_STREAM("after publishing rgb extrinsics");
    }

    if (this->intrinsic_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::INTRINSIC_CALIB))
    {
      auto buffer = frame->GetBuffer(ifm3d::buffer_id::INTRINSIC_CALIB);
      this->intrinsics_pub_.publish(ifm3d_to_intrinsics(buffer, head, getName()));
      NODELET_DEBUG_STREAM("after publishing rgb intrinsics");
    }

    if (this->rgb_info_stream_ && frame->HasBuffer(ifm3d::buffer_id::RGB_INFO))
    {
      auto buffer = frame->GetBuffer(ifm3d::buffer_id::RGB_INFO);
      this->rgb_info_pub_.publish(ifm3d_to_rgb_info(buffer, head, getName()));
      NODELET_DEBUG_STREAM("after publishing rgb info");
    }

    if (this->rgb_info_stream_ && frame->HasBuffer(ifm3d::buffer_id::INTRINSIC_CALIB))
    {
      auto buffer = frame->GetBuffer(ifm3d::buffer_id::INTRINSIC_CALIB);
      this->rgb_camera_info_pub_.publish(ifm3d_to_camera_info(buffer, head, rgb_img.height(), rgb_img.width(), getName()));
      NODELET_DEBUG_STREAM("after publishing camera info");
    }
}

void ifm3d_ros::CameraNodelet::Callback3D(ifm3d::Frame::Ptr frame){
    //
    // Pull out all the wrapped images so that we can release the "GIL"
    // while publishing
    //
    std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);
    lock.lock();
    ifm3d::Buffer xyz_img;
    ifm3d::Buffer confidence_img;
    ifm3d::Buffer distance_img;
    ifm3d::Buffer distance_noise_img;
    ifm3d::Buffer amplitude_img;
    ifm3d::Buffer extrinsics;
    ifm3d::Buffer rgb_img;

    NODELET_DEBUG_STREAM("start getting data");
    try
    {
      xyz_img =frame->GetBuffer(ifm3d::buffer_id::XYZ);
      confidence_img = frame->GetBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE);
      distance_img = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE);
      distance_noise_img = frame->GetBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE);
      amplitude_img = frame->GetBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE);
      extrinsics = frame->GetBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB);
      this->last_frame_time_ = frame->TimeStamps()[0];
      this->last_frame_local_time_ = ros::Time::now();

    }
    catch (const ifm3d::Error& ex)
    {
      NODELET_WARN_STREAM(ex.what());
    }
    catch (const std::exception& std_ex)
    {
      NODELET_WARN_STREAM(std_ex.what());
    }
    NODELET_DEBUG_STREAM("finished getting data");

    lock.unlock();

    //
    // Now, do the publishing
    //

    // Timestamps:
    this->head.stamp = ros::Time(
      std::chrono::duration_cast<std::chrono::duration<double,
      std::ratio<1>>>(frame->TimeStamps()[0].time_since_epoch()).count()
      );
    if ((ros::Time::now() - this->head.stamp) > ros::Duration(this->frame_latency_thresh_))
    {
      NODELET_INFO_ONCE("Camera's time and client's time are not synced");
      this->head.stamp = ros::Time::now();
    }
    // Header frame_id
    this->head.frame_id = this->frame_id_;

    if (this->amplitude_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::NORM_AMPLITUDE_IMAGE))
    {
      this->amplitude_pub_.publish(ifm3d_to_ros_image(amplitude_img, head, getName()));
      NODELET_DEBUG_STREAM("after publishing norm amplitude image");
    }

    if (this->confidence_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::CONFIDENCE_IMAGE))
    {
      this->conf_pub_.publish(ifm3d_to_ros_image(confidence_img, head, getName()));
      NODELET_DEBUG_STREAM("after publishing confidence image");
    }

    if (this->radial_distance_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_IMAGE))
    {
      this->distance_pub_.publish(ifm3d_to_ros_image(distance_img, head, getName()));
      NODELET_DEBUG_STREAM("after publishing distance image");
    }

    if (this->radial_distance_noise_stream_ && frame->HasBuffer(ifm3d::buffer_id::RADIAL_DISTANCE_NOISE))
    {
      this->distance_noise_pub_.publish(ifm3d_to_ros_image(distance_noise_img, head, getName()));
      NODELET_DEBUG_STREAM("after publishing distance noise image");
    }

    if (this->xyz_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::XYZ))
    {
      this->cloud_pub_.publish(ifm3d_to_ros_cloud(xyz_img, head, getName()));
      NODELET_DEBUG_STREAM("after publishing point cloud image");
    }


    //
    // publish extrinsics
    //
    if (this->extrinsic_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB))
    {
      auto buffer = frame->GetBuffer(ifm3d::buffer_id::EXTRINSIC_CALIB);
      this->extrinsics_pub_.publish(ifm3d_to_extrinsics(buffer, head, getName()));
      NODELET_DEBUG_STREAM("after publishing depth extrinsics");
    }

    if (this->intrinsic_image_stream_ && frame->HasBuffer(ifm3d::buffer_id::INTRINSIC_CALIB))
    {
      auto buffer = frame->GetBuffer(ifm3d::buffer_id::INTRINSIC_CALIB);
      this->intrinsics_pub_.publish(ifm3d_to_intrinsics(buffer, head, getName()));
      NODELET_DEBUG_STREAM("after publishing depth intrinsics");
    }
}

// this is the helper function for retrieving complete pcic frames
bool ifm3d_ros::CameraNodelet::StartStream()
{
  bool retval = false;
  NODELET_INFO_STREAM("Start streaming frames");
  try
  {
    // need to implement a nice strategy for getting the actual times
    // from the camera which are registered to the frame data in the image
    // buffer.

    if (strcmp(this->imager_type_.c_str(), "3D") == 0)
    {
      fg_->Start(this->schema_mask_default_3d_);
      NODELET_DEBUG_STREAM("Framegabbber initialized with default 3D schema mask");
      fg_->OnNewFrame(std::bind(&ifm3d_ros::CameraNodelet::Callback3D, this, std::placeholders::_1));
    }

    else if (strcmp(this->imager_type_.c_str(), "2D") == 0)
    {
      fg_->Start(this->schema_mask_default_2d_);
      NODELET_DEBUG_STREAM("Framegabbber initialized with default 2D schema mask");
      fg_->OnNewFrame(std::bind(&ifm3d_ros::CameraNodelet::Callback2D, this, std::placeholders::_1));
    }

    else
    {
      NODELET_INFO_STREAM("Unknown imager type");
    }
    this->last_frame_local_time_ = ros::Time::now();

  }
  catch (const ifm3d::Error& ex)
  {
    NODELET_WARN_STREAM(ex.code() << ": " << ex.what());
    retval = false;
  }

  return retval;
}

void ifm3d_ros::CameraNodelet::Run()
{
  std::unique_lock<std::mutex> lock(this->mutex_, std::defer_lock);

  NODELET_DEBUG_STREAM("in CameraNodelet Run");
  ros::Duration(5.0).sleep();

  while (ros::ok() && (!this->InitStructures(this->pcic_port_)))
  {
    NODELET_WARN_STREAM("Could not initialize pixel stream! Re-Initializing");
    ros::Duration(this->timeout_tolerance_secs_).sleep();
  }

  NODELET_DEBUG_STREAM("after initializing the buffers and services");

  this->StartStream();
  NODELET_INFO_STREAM("Started the camera stream");

  while (ros::ok())
  {
    if ((ros::Time::now() - last_frame_local_time_).toSec() > this->timeout_tolerance_secs_)
    {
      if (!this->assume_sw_triggered_)
      {
        NODELET_WARN_STREAM("Timeout waiting for camera!");
        NODELET_WARN_STREAM("Attempting to restart framegrabber...");
        while (!this->InitStructures(this->pcic_port_))
        {
          NODELET_WARN_STREAM("Could not re-initialize pixel stream!");
          ros::Duration(1.0).sleep();
        }
        this->StartStream();
      }
    }
    else
    {
      ros::Duration(.001).sleep();
    }

    continue;
  }

  fg_->Stop();
}  // end: Run()

PLUGINLIB_EXPORT_CLASS(ifm3d_ros::CameraNodelet, nodelet::Nodelet)
