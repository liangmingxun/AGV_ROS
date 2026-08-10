# ArUco UDP → ROS1 Task 15（v1.2 / UDP v3）

这是当前可部署的软件版本。定位链路固定为：

```text
顶部相机（Windows）
  → DICT_4X4_50检测
  → 四个固定地面码在线建立米制world坐标系
  → 计算AGV/载荷原始world_T_tag
  → UDP v3
  → Robot1协议与标定身份校验
  → /camera/world/*_tag_pose + *_confidence
  → Task 15适配器（tag→车体/载荷、raw/filtered、置信度门、超时）
  → path_state_estimator（CooperativeState）
```

桥接程序本身不发布TF、CooperativeState或运动命令，也不改变任何实车
执行授权。

## 1. 已冻结的实际配置

| 项目 | 当前值 |
|---|---|
| ArUco字典 | `DICT_4X4_50` |
| AGV1 / AGV2 / AGV3 | ID `1 / 2 / 3` |
| 载荷 | ID `0`，接口保留，正式发送暂时禁用 |
| 地面固定码 | ID `8 / 11 / 15 / 30` |
| 地面码有效边长 | `0.150 m` |
| 车载码有效边长 | `0.080 m` |
| 车载码平面高度 | `0.225 m` |
| 图像几何 | `1280 × 1024`，禁止静默缩放 |
| Windows | `192.168.6.100:15000` |
| Robot1 | `192.168.6.101:15001` |

三车标记安装均为标记`+Y`指向车辆正前方，且位于纵向中心线。
Task 15使用的实测`tag→base_link`为：Robot1
`(0,-0.13274,+pi/2)`，Robot2/3均为`(0,+0.03333,+pi/2)`，
平移单位为米，角度单位为弧度。

Robot2为`192.168.6.102`，Robot3为`192.168.6.103`，但二者不直接接收
视觉UDP。

当前内参和OpenCV顺序的五个畸变参数来自
`GrabImage202607301908`，已写入`windows_sender/config.yaml`并直接使用。
全帧不先做`cv2.undistort`；几何函数通过`undistortPoints/solvePnP`
使用同一组畸变参数，避免重复矫正。

`GrabImage`原程序的坐标职责同样是在线解算`world_T_camera`并输出
`world_T_tag`，没有执行`tag→base_link`。本版本并不是修正它的坐标
语义，而是将其发送前的位姿平滑和速度估计移到Task 15职责边界之外，
同时换成当前CRC/session/seq、四实体和ROS冻结接口的UDP v3格式。

相机高度变化不需要改内参。四个固定地面码会重新解算
`world_T_camera`。但如果镜头焦距、对焦、传感器ROI、分辨率或镜头本身
发生变化，原内参不再自动有效。

## 2. 不测量四个地面码世界坐标时，世界系如何定义

四个地面码必须固定、共面、全部采用150 mm有效黑色边长：

- ID 8中心定义`world`原点；
- ID 8指向ID 11定义`+X`；
- ID 30位于右手系的`+Y`一侧；从相机向下看时，它必须在有向线段
  ID 8→ID 11的左侧，并与该线保持足够横向距离；
- ID 15作为第四个平面拟合和连续监测码。

不需要预先输入四个码的世界坐标。米制比例来自已知的150 mm码边长和
有效相机内参，所以可以测量车辆运动距离；但该坐标只相对于这组地面码，
并不是实验室既有坐标系中的绝对坐标。若以后需要对齐实验室坐标，再增加
一次刚体坐标系对齐即可。

启动时，四个地面码需同时稳定可见至少15帧且持续约1.5秒。锁定后程序
持续监测固定码；相机被碰动、固定码被移动或残差连续超限时会自动取消
标定并停止发送，直到重新锁定。程序不会通过翻转`+Y`来迁就放反的
ID 30，因为那会同时把`+Z`翻到地面以下并破坏225 mm高度平面。

## 3. Windows部署

复制完整`windows_sender`目录，安装MVS运行库和64位Python，然后执行：

```bat
cd windows_sender
python -m pip install -r requirements.txt
python aruco_udp_sender.py --config config.yaml --detect-only
```

`--detect-only`不做世界坐标解算、不发UDP，但会显示地面码8/11/15/30、
AGV码1/2/3和暂时禁用的载荷码0。先确认：

- 字典确为`4X4_50`；
- 图像实际输出为`1280×1024`；
- 八个ID没有混用；
- 四个地面码和运动区域都具有足够像素与清晰度。

正式只读运行：

```bat
python aruco_udp_sender.py --config config.yaml
```

也可以双击`run_detect_only.bat`或`run_sender.bat`。画面显示
`GROUND=locked`后才会开始发UDP。`logs`目录保存发送前的原始标记位姿。

载荷目前仍能在detect-only中识别ID 0，但正式包不发送它。测得载荷码
高度并在Task 15填写真实`tag_to_target_*`后，才可同时修改两端高度并将
`entities.load.enabled`改为`true`。

## 4. Robot1部署与启动

当前实际开发工作树为：

```text
/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
```

桥接包已经在该工作树的`src/multi_agv_vision_bridge`中对齐。使用：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.6.101:11311
export ROS_IP=192.168.6.101
roscore
```

保持`roscore`终端运行，另开终端：

```bash
cd /home/etlab/AGV_ROS/.worktrees/platform-foundation-linux
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://192.168.6.101:11311
export ROS_IP=192.168.6.101
roslaunch multi_agv_vision_bridge vision_udp_bridge.launch
```

再开一个终端查看：

```bash
rostopic echo /vision/aruco/diagnostics
rostopic echo /vision/aruco/calibration_epoch
rostopic echo /camera/world/agv1_tag_pose
rostopic echo /camera/world/agv1_confidence
rostopic echo /vision/aruco/agv1_detected
```

移动ID 1时，`agv1_tag_pose.pose.position.x/y`和姿态四元数会变化；
`agv1_confidence`主要反映图像几何和重投影质量，不是位置值。

## 5. ROS接口与职责

四组冻结输入为：

```text
/camera/world/agv1_tag_pose       geometry_msgs/PoseStamped
/camera/world/agv2_tag_pose       geometry_msgs/PoseStamped
/camera/world/agv3_tag_pose       geometry_msgs/PoseStamped
/camera/world/load_tag_pose       geometry_msgs/PoseStamped
/camera/world/agv1_confidence     std_msgs/Float64
/camera/world/agv2_confidence     std_msgs/Float64
/camera/world/agv3_confidence     std_msgs/Float64
/camera/world/load_confidence     std_msgs/Float64
/vision/aruco/calibration_epoch   std_msgs/UInt64（锁存）
```

Windows和UDP桥均不执行标记到目标刚体的变换。现有Task 15
`camera_pose_adapter_node`继续负责：

- `tag_to_target_x/y/yaw`；
- 原始车体位姿和因果滤波位姿分离；
- confidence门；
- 时间戳、新鲜度和超时失效；
- 向`path_state_estimator_node`提供输入，由后者形成`CooperativeState`。

这些接口已经存在。
`.worktrees/platform-foundation-linux/src/multi_agv_bringup/config/localization_camera.yaml`
已写入三车实测`tag_to_target_*`；载荷组仍是零值占位。
`calibration_authorized=false`、`extrinsics_frozen=false`继续保持，直到三车旋转
中心验证、载荷禁用门和配置归档完成，不会因参数写入而自动
授权实车运行。

每路桥接位姿的`frame_id`采用`world@xxxxxxxx`并携带同一世界坐标代次。
ROS1会自动改写`header.seq`，因此它只保留普通消息流水号。地面重新锁定或Windows
发送会话更换后，Task 15会清空四路滤波与`CooperativeState`历史；旧代次
迟到消息被拒绝。正式运行不允许中途发生代次变化。

正式相机记录前，在Windows生成可复现清单：

```bat
python create_manifest.py --config config.yaml --test-stage task15
```

清单包含配置完整快照、配置SHA256以及发送器几何/协议源码SHA256。将JSON
复制到Robot1，并作为`experiment.launch windows_sender_manifest:=...`
参数归档。

## 6. 合成网络包

Windows可在不接相机时验证UDP/ROS接口：

```bat
python send_test_packet.py --config config.yaml --scenario normal --count 100 --rate 10
```

常用异常场景：

```bat
python send_test_packet.py --config config.yaml --scenario bad_crc
python send_test_packet.py --config config.yaml --scenario wrong_aruco
python send_test_packet.py --config config.yaml --scenario wrong_plane
python send_test_packet.py --config config.yaml --scenario out_of_order
python send_test_packet.py --config config.yaml --scenario omit_entity --omit-entity load
python send_test_packet.py --config config.yaml --scenario empty
```

Robot1除CRC、源端点、序号、时间、四实体映射外，还会核对：

- 标定模式必须为`ground_aruco_dynamic`；
- 地面ID顺序必须为`[8,11,15,30]`；
- 原点/X轴/Y侧角色必须为`8/11/30`；
- 地面码边长必须为`0.150 m`；
- AGV平面高度必须为`0.225 m`；
- 在线标定残差和样本数满足本地限制。

旧单应性包即使字段格式相似，也无法通过新的地面标定身份校验。

## 7. 文件结构

```text
windows_sender/
  aruco_udp_sender.py       Windows主程序
  ground_reference.py      四固定码外参、平面与实体SE(2)几何
  camera_sources.py        MVS/OpenCV取流
  protocol.py              UDP v3/CRC
  send_test_packet.py      合成网络包
  config.yaml              当前冻结配置

ros1_ws_src/multi_agv_vision_bridge/
  scripts/vision_udp_bridge.py
  src/multi_agv_vision_bridge/{protocol.py,validation.py}
  config/vision_udp_bridge.yaml
  launch/vision_udp_bridge.launch
```

旧的单应性点击标定工具及运行路径已经删除，不再是当前方案的一部分。
`GrabImage`四固定码几何是当前实现的算法来源，但其旧`robots`状态包
格式不直接接入现在的ROS桥。

## 8. 仍需真实测量的项目

- 四个AGV码到各自`base_link`的完整SE(2)刚体变换；
- 载荷码高度和`tag→load center`变换；
- 固定码粘贴后对距离、方向和重复性的物理尺检；
- 当前相机/镜头/焦距/分辨率组合下的实际定位误差；
- Windows与Robot1时钟同步后，是否从`receipt`切换到
  `capture_synced`；
- 最终工作区边界和执行授权。

软件测试通过不等于完成物理标定，也不自动授权底盘运动。
