# 基于CostMap2d(ROS1)的混合A*全局规划器和Teb局部规划器
## 主要工作内容
1. 参考 [nav2_smac_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner)，实现基于CostMap2d(ROS1)的混合A*全局规划器；
2. 参考 [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)，实现基于CostMap2d(ROS1)的TEB局部规划器；
3. 参考humble版本中的混合A*算法的实现，使用cached A* 算法，加速Heuristic的计算；
4. 基于nanoflann的kd-tree(radius search)加速障碍物查询，同时过滤调非必要的障碍物，进一步加速TEB的计算效率(100hz, Intel® Core™ i9-14900HX × 32, 默认配置)

## 主要依赖
- ubuntu20.04 + ros noetic

```bash
sudo apt install libeigen3-dev
sudo apt install libopencv-dev
sudo apt install libyaml-cpp-dev
sudo apt install ros-noetic-costmap-2d
sudo apt install libompl-dev
sudo apt install libsuitesparse-dev
sudo apt install ros-noetic-libg2o
```

## 便于运行

```bash
catkin_make
source devel/setup.bash
roslaunch path_follower follower.launch
```
在rviz中，通过2D Pose Estimate 发布起点，通过2D Nav Goal发布终点。

## 参考列表
1. [nav2_smac_planner](https://github.com/ros-navigation/navigation2/tree/main/nav2_smac_planner)
2. [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner)
3. [nanoflann](https://github.com/jlblancoc/nanoflann)
