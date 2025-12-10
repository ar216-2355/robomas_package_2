```bash
sudo ip link set can0 up type can bitrate 1000000
```

```bash
ros2 run robomas_package_2 sender --ros-args --params-file ./src/robomas_package_2/sender_params.yaml
```

```bash
ros2 run robomas_package_2 receiver
```

```bash
ros2 topic pub -1 /motor_cmd_array robomas_package_2/MotorCmdArray "
cmds:
- {id: 4, mode: 1, value: 300}
"
```