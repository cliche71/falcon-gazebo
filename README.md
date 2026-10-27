# FALCON Gazebo本地主线：v0.2.0-time-consistent

主用工作区固定为`/home/ljz/falcon_gazebo_repair_ws`。这是已批准Gazebo接入与真实历史快照的重建导入，不是纯官方源码。规划源码作为普通文件跟踪在src/FALCON，原Git历史单独集中归档。

## 日常启动

```bash
cd /home/ljz/falcon_gazebo_repair_ws
bash tools/start_office.sh gui:=true
```

默认配置：**original_a (0.5 / 0.7)**，显式official_compatible深度，启动后Gazebo原样暂停；没有自动prepare、start或探索倒计时。需要原A避障配置可使用`bash tools/start_office.sh profile:=original_a gui:=true`。不要使用历史reports中的旧启动脚本。

新终端按原流程手动准备/开始/停止，使用与launch相同的ROS_MASTER_URI：

```bash
cd /home/ljz/falcon_gazebo_repair_ws
source env.sh
rosservice call /gazebo/unpause_physics '{}'
rosservice call /native/prepare '{}'
# 等待/native/bridge_status显示READY后，由用户单次调用：
rosservice call /native/start '{}'
# 用户显式停止：
rosservice call /native/stop '{}'
```

本轮整理只做未prepare/start的启动检查，不安排新的探索。日志默认logs/office_<时间>/，可在启动前设置FALCON_LOG_DIR；日志和构建产物不入Git。

## 构建与版本

本机沿用已验证A devel（v2/v3）；没有为整理重编替换程序。旧B build缓存已在归档后删除。以后需要构建时执行`bash tools/build.sh`，从当前源码全新配置；切换到v1或其他源码后必须重新构建，不能把仍在devel的A程序冒称v1。原检查点在统一归档可恢复。

必需：ROS Noetic、现有RotorS/mav_comm、PCL/Eigen/NLopt、Gazebo和本接入插件；v2/v3还使用固定观察控制/model支撑。源、固定版本、哈希和构建方法见docs/DEPENDENCIES.md；不要删除falcon_turn_observation_ws。上游可选PCD/GIF/预编译库不入Git，清单与获取方式在config/resources_manifest.json和tools/fetch_optional_resources.py；当前Gazebo几何world/model本身完整跟踪。

三版来源见docs/VERSION_PROVENANCE.md，限制见docs/KNOWN_LIMITATIONS.md，原始记录与删除报告见docs/LOCAL_ARCHIVE_INDEX.md；用户后续手动推送见docs/MANUAL_PUSH.md。main最终指向第三次导入；此处不推送。
