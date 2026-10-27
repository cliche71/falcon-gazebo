// Passive observation of the SAME DepthCamera callback used by OpenNI.
// No SetActive, camera setters, native transport subscription or buffer writes.
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>
#include <iomanip>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

namespace gazebo {
class NativeRawDepthObserver : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!sensor_ || !ros::isInitialized() || !sensor_->DepthCamera()) {
      gzerr << "NativeRawDepthObserver requires DepthCamera and initialized ROS\n";
      return;
    }
    frame_ = sdf->Get<std::string>("frameName");
    node_.reset(new ros::NodeHandle());
    image_pub_ = node_->advertise<sensor_msgs::Image>("/native/observations/depth_raw", 2);
    meta_pub_ = node_->advertise<std_msgs::String>("/native/observations/depth_metadata", 1, true);
    config_pub_ = node_->advertise<std_msgs::String>("/native/observations/depth_sensor_sdf", 1, true);
    const auto camera = sensor_->DepthCamera();
    const auto parent = sdf->GetParent();
    const auto clip = parent->GetElement("camera")->GetElement("clip");
    std::ostringstream meta;
    meta << std::setprecision(17);
    meta << "{\"sensor\":\"" << sensor_->ScopedName() << "\",\"frame_id\":\"" << frame_
         << "\",\"near\":" << clip->Get<double>("near") << ",\"far\":" << clip->Get<double>("far")
         << ",\"render_near_float\":" << camera->NearClip() << ",\"render_far_float\":" << camera->FarClip()
         << ",\"width\":" << camera->ImageWidth() << ",\"height\":" << camera->ImageHeight()
         << ",\"hfov\":" << camera->HFOV().Radian() << ",\"vfov\":" << camera->VFOV().Radian()
         << ",\"projection_matrix\":[";
    const auto projection = camera->ProjectionMatrix();
    for (unsigned int r = 0; r < 4; ++r) for (unsigned int c = 0; c < 4; ++c) {
      if (r || c) meta << ',';
      meta << projection(r, c);
    }
    meta << "],\"source\":\"DepthCamera::ConnectNewDepthFrame\","
         << "\"stamp_source\":\"DepthCameraSensor::LastMeasurementTime\","
         << "\"unit\":\"optical_z_meters\",\"conversion\":\"memcpy_float32_only\"}";
    std_msgs::String message;
    message.data = meta.str();
    meta_pub_.publish(message);
    message.data = (parent ? parent : sdf)->ToString("");
    config_pub_.publish(message);
    connection_ = camera->ConnectNewDepthFrame(std::bind(
        &NativeRawDepthObserver::Frame, this, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));
  }
 private:
  void Frame(const float* data, unsigned int width, unsigned int height,
             unsigned int, const std::string&) {
    if (!image_pub_.getNumSubscribers()) return;
    const auto stamp = sensor_->LastMeasurementTime();
    sensor_msgs::Image message;
    message.header.stamp = ros::Time(stamp.sec, stamp.nsec);
    message.header.frame_id = frame_;
    message.width = width;
    message.height = height;
    message.encoding = "32FC1";
    const uint16_t marker = 1;
    message.is_bigendian = *reinterpret_cast<const uint8_t*>(&marker) == 0;
    message.step = width * sizeof(float);
    message.data.resize(message.step * height);
    std::memcpy(message.data.data(), data, message.data.size());
    image_pub_.publish(message);
  }
  sensors::DepthCameraSensorPtr sensor_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher image_pub_, meta_pub_, config_pub_;
  event::ConnectionPtr connection_;
  std::string frame_;
};
GZ_REGISTER_SENSOR_PLUGIN(NativeRawDepthObserver)
}  // namespace gazebo
