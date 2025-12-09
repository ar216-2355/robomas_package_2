ros2 topic pub -1 /motor_cmd_array robomas_package_2/MotorCmdArray "
cmds:
- {id: 4, mode: 1, value: 300}
"

ros2 topic pub -1 /motor_cmd_array robomas_package_2 MotorCmdArray "
cmds:
- {id: 4, mode: 1, value: 0}
"

ros2 run robomas_package_2 sender can0 --ros-args -p speed_gains_4:='[100.0,0.0,0.0,10000.0,10000.0]'