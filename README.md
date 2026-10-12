# gazebo_line_demo

Gazebo 里的一段马拉松赛道，用来运行切线弧跟踪。

赛道长约 114 米，有两段缓弯。跟踪节点和每秒 3.5 米的参数来自 [marathongo](https://github.com/landitbot/marathongo) 里的 `tangent_arc_navigation`。本仓库只包含仿真赛道和传感器话题，不包含那套跟踪程序。

传感器按比赛配置的话题发出：

- 32 线雷达：`/rslidar_points`，车体点云 `/current_scan_body`
- 惯导：`/imu/data`
- RTK：`/gnss_ksxt`

车的位置用仿真真值转发到 `/odometry`。边线按赛道几何发出。底盘是差速车，只跟踪速度指令。

## 环境

Ubuntu 20.04，ROS Noetic，Gazebo 11。工作空间里还需要能编译并运行：

- `vo_navigation`
- `speed_controller`
- `sim_interface`
- `costmap_converter`

这些包在 marathongo 的 `tangent_arc_navigation` 目录中。

## 编译

把本仓库放到该工作空间的 `src` 目录下，然后编译：

```bash
cd ~/marathongo/tangent_arc_navigation
catkin_make --pkg gazebo_line_demo
source devel/setup.bash
```

## 运行

```bash
source /opt/ros/noetic/setup.bash
source ~/marathongo/tangent_arc_navigation/devel/setup.bash
roslaunch gazebo_line_demo line_follow.launch gui:=true
```

无界面时把最后一行换成 `roslaunch gazebo_line_demo line_follow.launch gui:=false`。

## 许可

Apache License 2.0。详见 `LICENSE`。
