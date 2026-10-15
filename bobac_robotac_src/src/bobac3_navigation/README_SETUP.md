# Jazzy 独立构建与局部验证

目标为 Ubuntu 24.04 / ROS 2 Jazzy。课程镜像和公共启动环境由 A 维护；本包所需运行依赖已列入 `package.xml`，包括 MPPI、SmacPlanner、velocity smoother、Nav2 BT、rclpy、TF 和 Cyclone DDS。

所有操作使用独立目录，避免共享 build/install/log 和 `.runtime`：

```bash
cd /home/nvidia/桌面/Isaac4.5-6.0_ProjectAdaptation
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/bobac-nav-c/ros-logs
colcon --log-base /tmp/bobac-nav-c/log build \
  --base-paths bobac_robotac_src/src/bobac3_navigation \
  --packages-select bobac3_navigation \
  --build-base /tmp/bobac-nav-c/build --install-base /tmp/bobac-nav-c/install
source /tmp/bobac-nav-c/install/setup.bash
ros2 launch bobac3_navigation bobac3_navigation.launch.py --show-args
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_config.py
```

BT 注册、XML 和端口的离线检查（需要已安装的 BehaviorTree.CPP/tinyxml2 开发文件和 g++）：

```bash
g++ -std=c++17 bobac_robotac_src/src/bobac3_navigation/tests/bt_registration_check.cpp \
  -I/opt/ros/jazzy/include -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib \
  -lbehaviortree_cpp -ltinyxml2 -o /tmp/bobac-nav-c/bt_registration_check
/tmp/bobac-nav-c/bt_registration_check \
  /tmp/bobac-nav-c/install/bobac3_navigation/share/bobac3_navigation/behavior_trees/navigate_to_pose_no_spin.xml \
  /opt/ros/jazzy/lib
```

此检查加载 10 个实际 Nav2 BT 插件，注册树并校验端口；不会实例化动作节点或发送目标。它不能代替 lifecycle configure 中的树实例化或真实导航执行。

relay 和诊断逻辑局部检查：

```bash
export ROS_DOMAIN_ID=44
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 仅本次局部测试使用：loopback、禁用发现，不连接共享仿真或机器人。
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
python3 bobac_robotac_src/src/bobac3_navigation/tests/test_navigation_diagnostics.py
ros2 run bobac3_navigation navigation_readiness.py --timeout 2 --report /tmp/bobac-nav-c/readiness-negative.json
# 上一条无数据时预期退出 1，列出缺失数据、TF、lifecycle、action 和底盘接口。
unset CYCLONEDDS_URI
```

局部测试只发布合成 LaserScan；diagnostic 正向判定使用 mock。不会发布 `/cmd_vel` 或启动 Nav2。真实验收前必须取消上述测试用 DDS 隔离配置，保留 A 的课程 DDS 设置。

安装目录应包含 `share/bobac3_navigation/{launch,params,maps,rviz2,behavior_trees}` 和 `lib/bobac3_navigation/{laser_qos_relay.py,navigation_readiness.py}`。容器内应在 A 指定的 workspace overlay 上重复独立构建与检查；宿主机结果不等于容器验收。运行入口和只读诊断见 [README.md](README.md)。
