# iRobot® Create® 3 Simulator

[![License](https://img.shields.io/github/license/iRobotEducation/create3_sim)](https://github.com/iRobotEducation/create3_sim/blob/main/LICENSE)

This is a ROS 2 simulation stack for the [iRobot® Create® 3](https://edu.irobot.com/create3) robot, modified for multiple robot instances.

Have a look at the [Create® 3 documentation](https://iroboteducation.github.io/create3_docs/) for more details on the ROS 2 interfaces exposed by the robot.

### How to run

#### Simple robot mover
```bash
. install/setup.zsh && ros2 launch irobot_create_multi create3_gazebo.launch.py namespace:=robot1 spawn_dock:=false
```