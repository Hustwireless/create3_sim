# iRobot® Create® 3 Simulator

[![License](https://img.shields.io/github/license/iRobotEducation/create3_sim)](https://github.com/iRobotEducation/create3_sim/blob/main/LICENSE)

This is a ROS 2 simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot, modified for multiple robot instances.

Have a look at the [Create® 3 documentation](https://iroboteducation.github.io/create3_docs/) for more details on the ROS 2 interfaces exposed by the robot.

### How to run

#### Launch two robots
```bash
$ . install/setup.zsh && ros2 launch irobot_create_multi create3_gazebo.launch.py namespace:=robot1
```

```bash
$ . install/setup.zsh && ros2 launch irobot_create_multi create3_spawn.launch.py namespace:=robot2 x:=1.0
```

#### Send goals to Robot Walk action server
```bash
 $ . install/setup.zsh && ros2 action send_goal /robot1/walk create3_walk_msgs/action/Walk "{explore_duration:{sec: 100, nanosec: 0}, max_runtime:{sec: 120,nanosec: 0}}"
```

```bash
$ . install/setup.zsh && ros2 action send_goal /robot2/walk create3_walk_msgs/action/Walk "{explore_duration:{sec: 100, nanosec: 0}, max_runtime:{sec: 120,nanosec: 0}}"
```

Certainly! Here's a revised version of the section for your `README.md`:

### Discussion

- The current implementation, developed for ROS Humble, evolves from `createCoverage`. It utilizes a state machine to alternate behaviors among `navigation`, `estop`, `dock`, `undock`, and `rotate`. 
- `Navigation` leverages the [create3 API's `Drive Goals`](https://iroboteducation.github.io/create3_docs/api/drive-goals/) for movement directives.
- To deploy two identical robots, inclusive of all controllers and sensors, it would be good to spawn each robot's nodes and topics within distinct namespaces.
- An ongoing issue with ROS Galactic involves the lack of namespace support in several dependent packages. For details, refer to this [GitHub issue](https://github.com/ros-controls/gazebo_ros2_control/issues/37). Addressing this requires modifications to the `ros2_control`, `ros2_controllers`, and `gazebo_ros_control` packages.
- For ROS Humble, specifically regarding the correct namespace allocation for gazebo ros plugins, modifications to the `gazebo_ros` package appears to be necessary as well, which could fix this [Github issue](https://github.com/iRobotEducation/create3_sim/issues/215).

### Known Issues
- Sometimes when robot climbs up to the docking station, it stops moving afterwards. That's because the docking station mechanism has locked the robot's chassis.
- If the navigation velocity was set a bit larger, the robots cannot stop in time before cliding with each other.
