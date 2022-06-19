# Sentinel_Robomaster

Master Thesis project

## Pre-requisites

### ROS2
Install a current version of ROS2 (foxy or galactic), following the official instructions. and then install colcon

```bash
sudo apt install python3-colcon-common-extensions
```

### Robomaster ROS
Install [this repository](https://github.com/jeguzzi/robomaster_ros) that contains a ROS2 driver for the DJI Robomaster family of robots (EP and S1). 

### Robomaster Sim
Install [this repository](https://github.com/jeguzzi/robomaster_sim) that contains a library that emulates the firmware and remote API server of DJI Robomaster S1 and DJI Robomaster EP robots. They also implement a plugin for CoppeliaSim that simulates most of the functionality of the real robots.

### CoppeliaSim
To compile and then use the CoppeliaSim plugin you need ... [CoppeliaSim](https://www.coppeliarobotics.com).
Download the latest release. Export the location where you place it.

```bash
export COPPELIASIM_ROOT_DIR=<path to the folder containing the programming subfolder>
```
which on Linux is the root folder that you download, while on MacOs is `/Applications/coppeliaSim.app/Contents/Resources`, if you install the app to the default location.

## Usage
Use one of the two launch files {single_robot|multirobot}.launch to launch the scenario  and the robot model.

### Launch files
For the scenario of one single robot there's a single launch file that needs to be used, while for the scenarios where there are two robots two launch files should be launched.

```bash
ros2 launch sentinel single_robot.launch

ros2 launch sentinel multirobot_sentinel.launch 
ros2 launch sentinel multirobot_follower.launch
```

#### Arguments
The launch files can accept a list of arguments:

```bash
ros2 launch sentinel {single_robot|multirobot}.launch <key_1>:=<value_1> <key_2>:=<value_2> ...
```

| key              | type    | valid values              | default | description                                                                                           |
| ---------------- | ------- | ------------------------- | ------- | ----------------------------------------------------------------------------------------------------- |
| name             | string  | valid ROS names           | ''      | a name used as ROS namespace                                                                          |
| serial_number    | string  | 8 character ascii strings | ''      | the serial number of the robot, leave empty to connect to the first robot found                             |
| sim | bool |             | false     | Indicate if the launch file will be run in a simulation environment or not                    |
| movement_node        | string  | action_client, speed_controller           | speed_controller     | The node used for motion control of the robot        |
| anomaly_pub    | string  | on, off  | on  | the node Anomaly_publisher will be used for the simulated anomaly message    |
| linear_speed        | float    |                           | 0.5   | target linear speed for the robot               |
| angular_speed       | float    |                           | 0.55   | target angular speed for the robot             |
| teleop | string    |   on, off                       | off   | whether teleop will be used or not         |
| path_type        | string    |   simple, complex                       | simple    | type of path that the robots will follow                    |


#### Multirobot-specific Configurations
| gap_dist | float |             | 1.0    | Indicate the target gap distance that the two robots should keep                    |
| gap_theta | float |             | 0.52     | Indicate the target theta gap between the two robots                    |
