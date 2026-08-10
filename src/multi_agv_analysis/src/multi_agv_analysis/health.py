"""Read-only fleet inventory and ROS graph health checks."""

import ipaddress


ROLES = ("robot1", "robot2", "robot3", "windows_camera")


def validate_inventory(inventory):
    issues = []
    hosts = inventory.get("hosts", {})
    for role in ROLES:
        entry = hosts.get(role)
        if not isinstance(entry, dict):
            issues.append("missing host role: {}".format(role))
            continue
        try:
            ipaddress.ip_address(entry.get("ip", ""))
        except ValueError:
            issues.append("invalid IP for {}".format(role))
    ips = [hosts[role]["ip"] for role in ROLES if role in hosts]
    if len(ips) != len(set(ips)):
        issues.append("host IP addresses must be unique")
    if inventory.get("ros_master_role") != "robot1":
        issues.append("ros_master_role must remain robot1")
    if inventory.get("robot_count") != 3:
        issues.append("robot_count must be three")
    return {"valid": not issues, "issues": issues, "hosts": hosts}


def audit_ros_graph(publishers, subscribers, expect_commands):
    issues = []
    publisher_map = dict(publishers)
    subscriber_map = dict(subscribers)
    for index in range(1, 4):
        for suffix in ("chassis_feedback", "capability_report", "odom", "imu"):
            topic = "/agv{}/{}".format(index, suffix)
            count = len(publisher_map.get(topic, []))
            if count != 1:
                issues.append(
                    "{} publisher count is {}, expected 1".format(
                        topic, count))
        command = "/agv{}/chassis_command".format(index)
        command_publishers = publisher_map.get(command, [])
        expected = 1 if expect_commands else 0
        if len(command_publishers) != expected:
            issues.append(
                "{} publisher count is {}, expected {}".format(
                    command, len(command_publishers), expected))
        if any("move_base" in node for node in command_publishers):
            issues.append("move_base has forbidden command authority")
        if not subscriber_map.get(command):
            issues.append("{} has no subscriber".format(command))
    cooperative = publisher_map.get("/multi_agv/cooperative_state", [])
    if len(cooperative) != 1:
        issues.append(
            "cooperative_state publisher count is {}, expected 1".format(
                len(cooperative)))
    return {
        "valid": not issues,
        "issues": issues,
        "expect_commands": bool(expect_commands),
    }
