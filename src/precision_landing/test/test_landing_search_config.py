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
