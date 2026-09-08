# roscar_arm_driver

众灵 24 路舵机板的最小 ROS 2 串口驱动。驱动只调用板内已烧录动作，不在主机侧重算舵机轨迹。

| 服务 | 板卡命令 | 动作 |
|---|---|---|
| `/roscar_arm/probe` | 无 | 只打开串口，不运动 |
| `/roscar_arm/initialize` | `$DGS:1!` | 初始化 G0001 |
| `/roscar_arm/front_detect` | `$DGS:2!` | 前方检测 G0002 |
| `/roscar_arm/pickup_right` | `$DGT:3-7,1!` | 右侧药包拾取 G0003–G0007 |
| `/roscar_arm/pickup_left` | `$DGT:8-13,1!` | 左侧药包拾取 G0008–G0013 |
| GUI 板内动作 | `$DGT:14-17,1!` | 药包放置 G0014–G0017 |
| `/roscar_arm/stop` | `$DST!` | 停止动作 |

## car1 安装

Ubuntu 的 `brltty-udev` 会误认 `1a86:7523` CH340。先执行下面的可逆修复，再重新插拔 USB-TTL：

```bash
sudo systemctl mask --now brltty-udev.service brltty.service
sudo install -m 0644 install/roscar_arm_driver/share/roscar_arm_driver/udev/99-roscar-arm.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

构建和启动：

```bash
cd ~/roscar_humble_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roscar_arm_driver
source install/setup.bash
ros2 run roscar_arm_driver arm_driver_node
```

先做无运动检查：

```bash
ros2 service call /roscar_arm/probe std_srvs/srv/Trigger '{}'
```

确认机械臂周围无人、底座固定且可立即断电后，再调用动作服务。
