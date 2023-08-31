#ifndef ROS_WHEEL_HARDWARE_INTERFACE_HPP_
#define ROS_WHEEL_HARDWARE_INTERFACE_HPP_

#include <boost/assign/list_of.hpp>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/console.h>
#include <ros/ros.h>

class RosWheelHardwareInterface : public hardware_interface::RobotHW
{
public:
  RosWheelHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node, bool calculate_velocity,
                            std::vector<std::string> wheel_names_list, std::vector<std::string> joint_names_list);

  void updateJointsFromHardware(const ros::Duration& period);
  void writeCommandsToHardware();

private:
  ros::NodeHandle node_;
  ros::NodeHandle private_node_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  std::vector<std::string> wheel_names_list_;
  std::vector<std::string> joint_names_list_;
  std::vector<ros::Subscriber> wheel_angle_subs_;
  std::vector<ros::Subscriber> wheel_velocity_subs_;
  std::vector<ros::Publisher> wheel_vel_pubs_;
  std::vector<double> joint_angles_;
  std::vector<double> joint_velocities_;
  bool calculate_velocity_;

  struct Joint
  {
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;
  };

  std::vector<Joint> joints_;

  void registerControlInterfaces();
  void wheelVelocityCallback(const std_msgs::Float64::ConstPtr& msg, std::string wheel_name);
  void wheelAngleCallback(const std_msgs::Float64::ConstPtr& msg, std::string joint_name);
  int wheelIndexByName(std::string wheelName);
};

RosWheelHardwareInterface::RosWheelHardwareInterface(ros::NodeHandle node, ros::NodeHandle private_node,
                                                     bool calculate_velocity, std::vector<std::string> wheel_names_list,
                                                     std::vector<std::string> joint_names_list)
  : node_(node)
  , private_node_(private_node)
  , calculate_velocity_(calculate_velocity)
  , wheel_names_list_(wheel_names_list)
  , joint_names_list_(joint_names_list)
{
  for (int i = 0; i < (int)wheel_names_list_.size(); i++)
  {
    joint_angles_.push_back(0.0);
    joint_velocities_.push_back(0.0);
    Joint init_joint = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    joints_.push_back(init_joint);
    ros::Publisher wheel_velocity_pub =
        node_.advertise<std_msgs::Float64>("/" + wheel_names_list_[i] + "/target_velocity", 1);
    wheel_vel_pubs_.push_back(wheel_velocity_pub);
    if (!calculate_velocity_)
    {
      ros::Subscriber wheel_velocity_sub = node_.subscribe<std_msgs::Float64>(
          "/" + wheel_names_list_[i] + "/current_velocity", 1,
          boost::bind(&RosWheelHardwareInterface::wheelVelocityCallback, this, _1, wheel_names_list_[i]));
      wheel_velocity_subs_.push_back(wheel_velocity_sub);
    }
    ros::Subscriber wheel_angle_sub = node_.subscribe<std_msgs::Float64>(
        "/" + wheel_names_list_[i] + "/angle", 1,
        boost::bind(&RosWheelHardwareInterface::wheelAngleCallback, this, _1, wheel_names_list_[i]));
    wheel_angle_subs_.push_back(wheel_angle_sub);
  }

  registerControlInterfaces();
}

void RosWheelHardwareInterface::writeCommandsToHardware()
{
  for (int i = 0; i < (int)wheel_names_list_.size(); i++)
  {
    double angle_speed = joints_[i].velocity_command;
    std_msgs::Float64 wheel_vel_msg;
    wheel_vel_msg.data = angle_speed;
    wheel_vel_pubs_[i].publish(wheel_vel_msg);
  }
}

void RosWheelHardwareInterface::updateJointsFromHardware(const ros::Duration& period)
{
  for (int i = 0; i < (int)wheel_names_list_.size(); i++)
  {
    double delta_wheel = joint_angles_[i] - joints_[i].position - joints_[i].position_offset;
    if (std::abs(delta_wheel) < 1)
    {
      joints_[i].position += delta_wheel;
      if (calculate_velocity_)
      {
        joints_[i].velocity = delta_wheel / period.toSec();
      }
    }
    else
    {
      joints_[i].position_offset += delta_wheel;
    }
    if (!calculate_velocity_)
    {
      joints_[i].velocity = joint_velocities_[i];
    }
  }
}

void RosWheelHardwareInterface::registerControlInterfaces()
{
  for (unsigned int i = 0; i < joint_names_list_.size(); i++)
  {
    hardware_interface::JointStateHandle joint_state_handle(joint_names_list_[i], &joints_[i].position,
                                                            &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

void RosWheelHardwareInterface::wheelVelocityCallback(const std_msgs::Float64::ConstPtr& msg, std::string wheel_name)
{
  // _front_left_wheel_angle = msg.data;
  int wheel_index = wheelIndexByName(wheel_name);
  if (wheel_index < 0)
  {
    ROS_WARN("Received velocity for unknown wheel %s", wheel_name.c_str());
  }
  else
  {
    joint_velocities_[wheel_index] = msg->data;
  }
}

void RosWheelHardwareInterface::wheelAngleCallback(const std_msgs::Float64::ConstPtr& msg, std::string wheel_name)
{
  // _front_left_wheel_angle = msg.data;
  int wheel_index = wheelIndexByName(wheel_name);
  if (wheel_index < 0)
  {
    ROS_WARN("Received angle for unknown wheel %s", wheel_name.c_str());
  }
  else
  {
    joint_angles_[wheel_index] = msg->data;
  }
}

int RosWheelHardwareInterface::wheelIndexByName(std::string wheelName)
{
  std::vector<std::string>::iterator itr = std::find(wheel_names_list_.begin(), wheel_names_list_.end(), wheelName);
  if (itr != wheel_names_list_.end())
  {
    return std::distance(wheel_names_list_.begin(), itr);
  }
  else
  {
    return -1;
  }
}

#endif  // ROS_WHEEL_HARDWARE_INTERFACE_HPP_
