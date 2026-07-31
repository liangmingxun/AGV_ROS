from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
source = (
    ROOT
    / "ros1_ws_src"
    / "multi_agv_vision_bridge"
    / "scripts"
    / "vision_udp_bridge.py"
).read_text(encoding="utf-8")
config = (
    ROOT
    / "ros1_ws_src"
    / "multi_agv_vision_bridge"
    / "config"
    / "vision_udp_bridge.yaml"
).read_text(encoding="utf-8")


def test_frozen_message_types_and_topics():
    assert "PoseStamped" in source
    assert "Float64" in source
    assert "UInt64" in source
    assert "/vision/aruco/calibration_epoch" in source
    assert 'f"{self.frame_id}@{self.calibration_epoch_token:08x}"' in source
    for entity in ("agv1", "agv2", "agv3", "load"):
        assert f"/camera/world/{entity}_tag_pose" in config
        assert f"/camera/world/{entity}_confidence" in config
    assert "expected_source_ip: 192.168.6.100" in config
    assert "expected_source_port: 15000" in config
    assert "bind_ip: 192.168.6.101" in config
    assert "bind_port: 15001" in config


def test_forbidden_publish_authority_absent():
    for forbidden in (
        "PoseWithCovarianceStamped",
        "geometry_msgs.msg import Twist",
        "CooperativeState",
        "chassis_command",
        "cmd_vel",
        "tf2_ros",
        "TransformBroadcaster",
    ):
        assert forbidden not in source


def test_bridge_enforces_frozen_endpoint_and_calibration_limits():
    assert 'source != (self.expected_source_ip, self.expected_source_port)' in source
    assert "ground_reference_spec=self.ground_reference_spec" in source
    assert "calibration_limits=self.calibration_limits" in source
    assert "marker_ids: [8, 11, 15, 30]" in config
    assert "marker_length_m: 0.150" in config
    assert "max_validation_rmse_m" in config
    assert "min_inlier_ratio" in config
