# PM01_Marathon

Gazebo 马拉松路段，配合 [marathongo](https://github.com/landitbot/marathongo) 的 `tangent_arc_navigation` 使用。跟踪程序不在本仓库内。

赛道约 450 m，坡度约 1 度，弯道半径 60 m。行进线靠右，右侧车道宽 3.5 m，左侧另有 3.5 m 超车道。左右外沿是边界，中间虚线只作分道。速度参数沿用 `vo_navigation`，直道上限 3.5 m/s。定位使用 `glio_mapping`。

机器人使用众擎 PM01 教育版的 URDF 和外壳网格。关节保持默认站姿。骨盆高度约 0.82 m。头部和腰部相机、机身 IMU 按现有硬件发布。三维雷达装在躯干前方，主 RTK 天线在头顶正中，辅天线在其右侧 0.07 m。这两处是预留安装位，32 线点云和 `/gnss_ksxt` 已经从这里发布。前进由速度指令驱动。

## 话题

- `/rslidar_points`：32 线点云，坐标系 `laser`
- `/current_scan_body`：车体坐标系点云
- `/imu/data`：机身 IMU，200 Hz
- `/gnss_ksxt`：主天线对应的 RTK 服务
- `/camera/cam_head/color/image_raw`：头部相机
- `/camera/cam_torso/color/image_raw`：腰部相机
- `/Odometry`：`glio_mapping` 输出
- `/odometry_body`：补上线速度后送给跟踪
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
catkin_make --pkg PM01_Marathon
source devel/setup.bash
```

## 运行

```bash
source /opt/ros/noetic/setup.bash
source ~/marathongo/tangent_arc_navigation/devel/setup.bash
source ~/glio_ws/devel/setup.bash --extend
roslaunch PM01_Marathon line_follow.launch gui:=true
```

不打开窗口时使用 `gui:=false`。
