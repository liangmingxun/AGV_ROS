# ArUco UDP协议v3（Task 15）

## 固定网络

```text
Windows 192.168.6.100:15000 → Robot1 192.168.6.101:15001
```

Robot1同时核对真实UDP源IP/端口与包内`source_ip`。Robot2、Robot3不
直接接收视觉UDP。

## 信封

外层是规范化UTF-8 JSON与CRC32：

```json
{"body":{"...":"..."},"crc32":"8位十六进制"}
```

协议名为`multi_agv_aruco_pose`，版本为整数`3`，最大8192字节。v2和
旧版`robots`状态包必须拒绝；旧包内坐标仍是标记位姿，但包结构、滤波
位置和当前冻结接口不兼容。

## body必需字段

- `protocol`、`version`
- `session_id`、严格递增的`seq`
- `capture_time_unix_ns`、`send_time_unix_ns`
- `source_ip`、`frame_id=world`、`camera_alive=true`
- `frame_number`、`processing_ms`
- `ground_reference`
- `calibration`
- `detections`，长度0～4

`ground_reference`固定为：

```json
{
  "mode": "ground_aruco_dynamic",
  "marker_ids": [8, 11, 15, 30],
  "origin_id": 8,
  "x_axis_id": 11,
  "y_side_id": 30,
  "marker_length_m": 0.15,
  "sample_count": 60,
  "calibrated_at_unix_ns": 1780000000000000000
}
```

Robot1按本地配置逐项核对标定模式、ID顺序、角色和边长。

## 四个实体

| entity_id | ArUco ID | plane_id | 当前状态 |
|---|---:|---|---|
| `agv1` | 1 | `agv_marker_plane` | 启用 |
| `agv2` | 2 | `agv_marker_plane` | 启用 |
| `agv3` | 3 | `agv_marker_plane` | 启用 |
| `load` | 0 | `load_marker_plane` | 协议保留，Windows暂不发送 |

载荷是独立实体，不是第四台机器人。每个detection示例：

```json
{
  "entity_id": "agv1",
  "aruco_id": 1,
  "x_m": 0.214,
  "y_m": 0.083,
  "yaw_rad": 0.031,
  "quality": 0.93,
  "marker_area_px": 2810.0,
  "pixel_x": 412.3,
  "pixel_y": 336.8,
  "plane_id": "agv_marker_plane"
}
```

`x/y/yaw`是原始`world_T_tag`。标记`+X`由ArUco对象坐标确定：
解码角点顺序为`(-x,+y),(+x,+y),(+x,-y),(-x,-y)`。Windows不执行
`tag→base_link`或`tag→load center`。

## calibration

每个平面携带：

```text
<plane>_rmse_m
<plane>_max_error_m
<plane>_validation_rmse_m
<plane>_validation_max_error_m
<plane>_inlier_count
<plane>_total_count
<plane>_validation_count
<plane>_height_m
```

当前AGV高度为0.183 m。载荷高度暂为0.0 m占位，且不产生载荷检测。
Robot1按本地阈值复核残差、样本数、比例和高度。

## 时间与ROS输出

首次联调使用`receipt`，PoseStamped采用Robot1收包时刻。只有完成两机
对时与延迟证据后才使用`capture_synced`。

桥发布四路`/camera/world/<entity>_tag_pose`和四路
`/camera/world/<entity>_confidence`，同一检测先发布confidence再发布
PoseStamped。ROS输出`frame_id`为`world@xxxxxxxx`，后8位十六进制数由
`session_id + calibrated_at_unix_ns`生成，是非零世界坐标代次；同一次
地面锁定保持不变。`header.seq`由ROS1发布器管理，不承载协议语义。完整纳秒代次通过锁存话题
`/vision/aruco/calibration_epoch`（`std_msgs/UInt64`）发布。Task 15在
代次变化时清空滤波和状态历史，正式实验校验器会把一段运行中出现多个
代次判为无效。桥不发布TF、CooperativeState或运动命令。
