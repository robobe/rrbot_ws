```
ros2 run controller_manager spawner position_controller
```

```
ros2 topic pub --once \
/position_controller/commands \
std_msgs/msg/Float64MultiArray "{data: [0, 0]}"
```

```
ros2 run controller_manager spawner imu_sensor_broadcaster

```

```
ros2 run controller_manager spawner joint_state_broadcaster
```

```
ros2 run controller_manager spawner position_controller
ros2 run controller_manager spawner effort_controller

```


# Reference 
- [Any PID-based "controller_interface::ControllerInterface" implementations/examples for ROS2](https://answers.ros.org/question/398614/any-pid-based-controller_interfacecontrollerinterface-implementationsexamples-for-ros2/?answer=398657)
- [](https://answers.ros.org/question/395815/do-i-need-to-write-my-own-pid-controller-when-using-ros2_control/?sort=oldest)