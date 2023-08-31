
#include <chrono>
#include <functional>
#include <ros/callback_queue.h>

#include "ros_wheel_hardware_interface.h"

typedef boost::chrono::steady_clock time_source;

void controlLoop(RosWheelHardwareInterface& hardware, controller_manager::ControllerManager& cm,
                 time_source::time_point& last_time)
{
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  hardware.updateJointsFromHardware(elapsed);
  cm.update(ros::Time::now(), elapsed);
  hardware.writeCommandsToHardware();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_wheel_base");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  double control_frequency;
  bool calculate_velocity;
  std::vector<std::string> wheel_names_list;
  std::vector<std::string> joint_names_list;

  if (!private_node.hasParam("joint_names"))
  {
    ROS_ERROR("Can't run without joint_names parameter");
    return -1;
  }
  if (!private_node.hasParam("wheel_names"))
  {
    ROS_ERROR("Can't run without wheel_names parameter");
    return -1;
  }

  private_node.param<double>("control_frequency", control_frequency, 1);
  private_node.param<bool>("calculate_velocity", calculate_velocity, false);
  private_node.getParam("joint_names", joint_names_list);
  private_node.getParam("wheel_names", wheel_names_list);

  ROS_INFO("Control frequency set to %f", control_frequency);

  if (joint_names_list.size() == 0)
  {
    ROS_ERROR("joints list can't be empty");
    return -1;
  }

  if (wheel_names_list.size() == 0)
  {
    ROS_ERROR("wheels list can't be empty");
    return -1;
  }

  if (joint_names_list.size() != wheel_names_list.size())
  {
    ROS_ERROR("wheels and joints arrays must be of the same size");
  }

  for (int i = 0; i < (int)joint_names_list.size(); i++)
  {
    if (joint_names_list[i].length() == 0)
    {
      ROS_ERROR("joint name can't be empty");
      return -1;
    }
    if (wheel_names_list[i].length() == 0)
    {
      ROS_ERROR("wheel name can't be empty");
      return -1;
    }
    ROS_INFO("Adding joint %s for wheel %s", joint_names_list[i].c_str(), wheel_names_list[i].c_str());
  }

  RosWheelHardwareInterface hardware(node, private_node, calculate_velocity, wheel_names_list, joint_names_list);

  controller_manager::ControllerManager cm(&hardware, node);

  ros::CallbackQueue wheel_base_queue;
  ros::AsyncSpinner wheel_base_spinner(1, &wheel_base_queue);

  time_source::time_point last_time = time_source::now();

  ros::TimerOptions control_timer(ros::Duration(1 / control_frequency),
                                  boost::bind(controlLoop, std::ref(hardware), std::ref(cm), std::ref(last_time)),
                                  &wheel_base_queue);

  ros::Timer control_loop = node.createTimer(control_timer);

  wheel_base_spinner.start();
  ros::spin();
  return 0;
}
