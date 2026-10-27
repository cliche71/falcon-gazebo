#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Contact.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iomanip>
#include <sstream>

namespace gazebo {
class NativeContactObserver : public WorldPlugin {
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override {
    world_ = world;
    if (!ros::isInitialized()) {
      int count = 0;
      char **arguments = nullptr;
      ros::init(count, arguments, "native_contact_observer", ros::init_options::NoSigintHandler);
    }
    node_.reset(new ros::NodeHandle());
    contacts_ = node_->advertise<std_msgs::String>("/native/diagnostics/contacts", 100);
    registry_ = node_->advertise<std_msgs::String>("/native/diagnostics/collision_registry", 1, true);
    world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
    connection_ = event::Events::ConnectWorldUpdateEnd(std::bind(&NativeContactObserver::update, this));
  }

  void update() {
    auto model = world_->ModelByName("hummingbird");
    if (model && !registered_) {
      std::ostringstream stream;
      stream << "{\"stamp\":" << world_->SimTime().Double() << ",\"collisions\":[";
      bool first = true;
      for (const auto &link : model->GetLinks())
        for (const auto &collision : link->GetCollisions()) {
          if (!first)
            stream << ',';
          first = false;
          stream << '"' << collision->GetScopedName() << '"';
        }
      stream << "]}";
      std_msgs::String message;
      message.data = stream.str();
      registry_.publish(message);
      registered_ = true;
    }
    auto manager = world_->Physics()->GetContactManager();
    std::ostringstream stream;
    stream << std::setprecision(12) << "{\"stamp\":" << world_->SimTime().Double() << ",\"contacts\":[";
    bool first = true;
    for (unsigned int index = 0; index < manager->GetContactCount(); ++index) {
      auto contact = manager->GetContacts()[index];
      std::string first_name = contact->collision1->GetScopedName();
      std::string second_name = contact->collision2->GetScopedName();
      if (first_name.find("hummingbird::") == std::string::npos && second_name.find("hummingbird::") == std::string::npos)
        continue;
      for (unsigned int point = 0; point < contact->count; ++point) {
        if (!first)
          stream << ',';
        first = false;
        const auto &position = contact->positions[point];
        const auto &normal = contact->normals[point];
        const auto &force = contact->wrench[point].body1Force;
        stream << "{\"collision1\":\"" << first_name << "\",\"collision2\":\"" << second_name
               << "\",\"depth\":" << contact->depths[point] << ",\"position\":[" << position.X()
               << ',' << position.Y() << ',' << position.Z() << "],\"normal\":[" << normal.X() << ','
               << normal.Y() << ',' << normal.Z() << "],\"force1\":[" << force.X() << ',' << force.Y()
               << ',' << force.Z() << "]}";
      }
    }
    stream << "]}";
    std_msgs::String message;
    message.data = stream.str();
    contacts_.publish(message);
  }

private:
  physics::WorldPtr world_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Publisher contacts_, registry_;
  event::ConnectionPtr connection_;
  bool registered_ = false;
};
GZ_REGISTER_WORLD_PLUGIN(NativeContactObserver)
}
