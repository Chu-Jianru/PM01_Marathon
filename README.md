# gazebo_line_demo

Gazebo 马拉松路段，配合 [marathongo](https://github.com/landitbot/marathongo) 的 `tangent_arc_navigation` 使用。跟踪程序不在本仓库内。

赛道约 114 m，弯道半径 60 m。速度参数沿用 `vo_navigation`，直道上限 3.5 m/s。

## 话题

- `/rslidar_points`：32 线点云
- `/current_scan_body`：车体坐标系点云
- `/imu/data`
- `/gnss_ksxt`
- `/odometry`：仿真位姿
- `/central/smoothed_path`、`/left/smoothed_path`、`/right/smoothed_path`

## 依赖

Ubuntu 20.04，ROS Noetic，Gazebo 11。

同一工作空间需要：

- `vo_navigation`
- `speed_controller`
- `sim_interface`
- `costmap_converter`

## 编译

将本仓库放在 `tangent_arc_navigation/src` 下：

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

不打开窗口时使用 `gui:=false`。
