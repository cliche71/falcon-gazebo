// Visibility only: preserve physical links, collisions and all environment visuals.
#include <functional>
#include <memory>
#include <set>
#include <sstream>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderTarget.h>
#include <OGRE/OgreViewport.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gazebo {
class NativeDepthSelfFilter : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    auto depth = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(sensor);
    if (!depth || !depth->DepthCamera() || !ros::isInitialized()) return;
    camera_ = depth->DepthCamera();
    model_ = sdf->Get<std::string>("modelName");
    node_.reset(new ros::NodeHandle());
    pub_ = node_->advertise<std_msgs::String>("/native/observations/self_visibility", 1, true);
    connection_ = event::Events::ConnectPreRender(std::bind(&NativeDepthSelfFilter::Update, this));
  }
 private:
  void Mark(const rendering::VisualPtr& visual) {
    for (unsigned int i = 0; i < visual->GetChildCount(); ++i) Mark(visual->GetChild(i));
    marked_.insert(visual->Name());
    visual->SetVisibilityFlags(kSelfBit);
  }
  void Update() {
    auto root = camera_->GetScene()->GetVisual(model_);
    if (!root) return;
    Mark(root);
    unsigned int views = 0;
    auto targets = Ogre::Root::getSingleton().getRenderSystem()->getRenderTargetIterator();
    while (targets.hasMoreElements()) {
      auto target = targets.getNext();
      for (unsigned int i = 0; i < target->getNumViewports(); ++i) {
        auto view = target->getViewport(i);
        if (view->getCamera() == camera_->OgreCamera()) {
          view->setVisibilityMask(view->getVisibilityMask() & ~kSelfBit);
          ++views;
        }
      }
    }
    if (marked_.size() == reported_ && views == reported_views_) return;
    reported_ = marked_.size(); reported_views_ = views;
    std::ostringstream out;
    out << "{\"model\":\"" << model_ << "\",\"self_visual_bit\":" << kSelfBit
        << ",\"camera_viewports_filtered\":" << views
        << ",\"scope\":\"only viewports belonging to original depth camera; other camera masks unchanged\",\"visuals\":[";
    bool comma = false;
    for (const auto& name : marked_) { if (comma) out << ','; out << '"' << name << '"'; comma = true; }
    out << "]}";
    std_msgs::String message; message.data = out.str(); pub_.publish(message);
  }
  static constexpr uint32_t kSelfBit = 0x04000000;
  rendering::DepthCameraPtr camera_;
  event::ConnectionPtr connection_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher pub_;
  std::string model_;
  std::set<std::string> marked_;
  size_t reported_ = 0;
  unsigned int reported_views_ = 0;
};
GZ_REGISTER_SENSOR_PLUGIN(NativeDepthSelfFilter)
}
