ros_wheel_base
# ros_wheel_base

## Overview

A universal ROS base with hardware interface for mult-wheel ROS robots.
Provides a velocity interface between ROS controllers and possibly any hardware driver to control motors/wheels of a robot through a set of topics for every motor/wheel.

## Usage

### Getting the code

To use the node just check it out to your ROS workspace:
```
git clone https://github.com/belovictor/ros_wheel_base.git
```

### Configuration

Node is configured with the following configuration paramters:

| Parameter name                | Type     | Description                                                                        |
|-------------------------------|----------|------------------------------------------------------------------------------------|
| ```max_wheel_angular_speed``` | Float    | Maximum wheel angular speed in rad/s                                               |
| ```control_frequency```       | Float    | Control rate between controller and driver in Hz                                   |
| ```calculate_velocity```      | Boolean  | If set to true then interface will calculate feedback velocity to joint fron angle |
| ```wheel_names```             | String[] | Array of names of wheels which are then used to name topics corresponding to wheels|
| ```joint_names```             | String[] | Array of names of joints which are bind to corresponding wheels                    |

An example configuration for a 4 wheel robot is provided in config/ros_wheel_base.yaml. Every wheel have to have a uniq name and corresponds to the joint with uniq name.

### Running the node

```
roslaunch ros_wheel_base ros_wheel_base.launch
```

### How it works?

The node binds to controller through joints and then exposes several topics for every wheel to communicate with hardware driver.

| Topic name                         | Message type           | Direction | Description                             |
|------------------------------------|------------------------|-----------|-----------------------------------------|
|```<wheel name>/angle```            | ```std_msgs/Float64``` | Inboud    | Angle of the wheel in rad               |
|```<wheel name>/current_velocity``` | ```std_msgs/Float64``` | Inbound   | Current wheel angular velocity in rad/s |
|```<whee; name>/target_velocity```  | ```std_msgs/Float64``` | Outbound  | Target wheel angular velocity in rad/s  |

If hardware (or hardware driver) does not provide velocity feedback from the wheel and only provides angle information, you can set calculate_velocity configuration parameter to true and interface will calculate current velocity for every wheel based on it's angle changes.
