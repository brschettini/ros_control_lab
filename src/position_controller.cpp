#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace action_controller { 

class PositionActionController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  PositionActionController();

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    // get joint name from the parameter server
    std::string my_joint;
    if (!controller_nh.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
    return true;
  }

  void starting(const ros::Time& time) {}

  void stoping(const ros::Time& time) {}
  
  void update(const ros::Time& time, const ros::Duration& period) {}

 private:
  hardware_interface::JointHandle joint_;
  static const double gain_ = 1.25;
  static const double setpoint_ = 3.00;


};

PLUGINLIB_EXPORT_CLASS(action_controller::PositionActionController, controller_interface::ControllerBase)

} // namespace action_controllers
