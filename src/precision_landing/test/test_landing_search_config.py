from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_search_config_uses_calibrated_down_camera_extrinsic():
    config = yaml.safe_load((PACKAGE / "config/landing_search.yaml").read_text())
    transform = config["camera_to_body"]
    assert transform["translation_m"] == [
        0.07383116536212123,
        -0.03902001736630811,
        -0.13346814265233625,
    ]
    quaternion = transform["quaternion_xyzw"]
    assert math.isclose(sum(value * value for value in quaternion), 1.0, abs_tol=1e-9)
    assert config["mission"]["require_stage_gate"] is True
    assert config["topics"]["landing_trigger"] == "/UAV0/need_to_land"
    assert config["handoff"]["approach_height_m"] == 2.0


def test_search_launch_wires_mission_request_to_precision_landing_trigger():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    search_nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "landing_search_node"
    ]
    assert len(search_nodes) == 1
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in search_nodes[0].findall("param")
    }
    assert params["topics/landing_request"] == "$(arg mission_landing_request_topic)"
    assert params["topics/landing_trigger"] == "$(arg trigger_topic)"
    assert params["topics/marker_world"] == "$(arg landing_marker_world_topic)"
    assert params["topics/target_id"] == "$(arg target_id_topic)"


def test_front_hint_uses_d435_depth_tf_and_separate_coarse_topic():
    config = yaml.safe_load(
        (PACKAGE / "config/front_aruco_hint.yaml").read_text()
    )
    assert config["marker"]["size_m"] == 0.60
    assert config["marker"]["stable_frames"] == 5
    assert config["depth_validation"]["required"] is True
    assert config["frames"]["body"] == "UAV0/body"
    assert config["frames"]["output_world"] == "world"
    assert config["topics"]["hint_world"] == "/UAV0/landing/front_aruco_hint"
    assert config["topics"]["hint_world"] != config["topics"].get("marker_world")


def test_precision_launch_wires_optional_front_hint_without_final_marker_access():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "front_aruco_hint_node"
    ]
    assert len(nodes) == 1
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in nodes[0].findall("param")
    }
    assert params["topics/image"] == "$(arg front_image_topic)"
    assert params["topics/aligned_depth"] == "$(arg front_depth_topic)"
    assert params["topics/odometry"] == "$(arg odometry_topic)"
    assert params["topics/hint_world"] == "$(arg front_aruco_hint_topic)"
    assert "topics/marker_world" not in params
    assert "topics/landing_trigger" not in params


def test_precision_launch_combines_downward_debug_views():
    root = ET.parse(PACKAGE / "launch/precision_landing.launch").getroot()
    nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "landing_debug_image_mux_node"
    ]
    assert len(nodes) == 1
    params = {
        param.attrib["name"]: param.attrib["value"]
        for param in nodes[0].findall("param")
    }
    assert params == {
        "search_image_topic": "$(arg topic_prefix)/landing/search/debug_image",
        "precision_image_topic": "$(arg topic_prefix)/landing/debug_image",
        "trigger_topic": "$(arg trigger_topic)",
        "output_topic": "$(arg topic_prefix)/landing/combined_debug_image",
    }

    rviz_config = (
        PACKAGE.parent
        / "zyc_fuel_ws/src/FUEL/fuel_planner/plan_manage/config/traj.rviz"
    ).read_text()
    assert rviz_config.count(
        "Image Topic: /UAV0/landing/combined_debug_image"
    ) == 1
    assert "Name: landing_down_search_image" not in rviz_config
    assert "Name: precision_landing_image" not in rviz_config
