from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE / "src/autotrans_trajectory_arbiter_node.cpp").read_text()


def test_arbiter_latches_diff_for_every_search_landing_stage():
    for stage in (
        "SEARCH_OUTSIDE_LANDING",
        "SEARCH_OUTSIDE_QR",
        "APPROACH_LANDING",
        "LANDING",
    ):
        assert f'stage == "{stage}"' in SOURCE
    assert "if (diff_owned_ || !isDiffSearchStage(stage)) return;" in SOURCE
    assert "diff_owned_ = true" in SOURCE


def test_arbiter_aborts_stale_fuel_and_gates_both_inputs():
    assert "ACTION_ABORT" in SOURCE
    assert "if (diff_owned_) return;" in SOURCE
    assert "if (!diff_owned_) return;" in SOURCE


def test_uav0_launch_routes_fuel_and_diff_through_one_autotrans_input():
    root = ET.parse(PACKAGE / "launch/uav0_autotrans_controller.launch").getroot()
    args = {item.attrib["name"]: item.attrib["default"] for item in root.findall("arg")}
    nodes = {node.attrib["name"]: node for node in root.findall("node")}

    fuel_params = {
        item.attrib["name"]: item.attrib["value"]
        for item in nodes["fuel_autotrans_bridge"].findall("param")
    }
    diff_params = {
        item.attrib["name"]: item.attrib["value"]
        for item in nodes["uav0_diff_autotrans_reference_bridge"].findall("param")
    }
    arbiter_params = {
        item.attrib["name"]: item.attrib["value"]
        for item in nodes["autotrans_trajectory_arbiter"].findall("param")
    }

    assert args["pure_fuel_mode"] == "false"
    assert fuel_params["output_topic"] == (
        "$(eval '/UAV0/planning/autotrans_trajectory' if "
        "arg('pure_fuel_mode') else '/UAV0/fuel/autotrans_trajectory')"
    )
    assert nodes["uav0_diff_autotrans_reference_bridge"].attrib["unless"] == (
        "$(arg pure_fuel_mode)"
    )
    assert nodes["autotrans_trajectory_arbiter"].attrib["unless"] == (
        "$(arg pure_fuel_mode)"
    )
    assert diff_params["input_topic"] == "/drone_0_planning/trajectory"
    assert diff_params["output_topic"] == "/UAV0/diff/autotrans_trajectory"
    assert arbiter_params["output_topic"] == "/UAV0/planning/autotrans_trajectory"
    assert arbiter_params["stage_topic"] == "/UAV0/mission/task_status"

    controller_include = next(
        item
        for item in root.findall("include")
        if "quad_wind_mpc_controller.launch" in item.attrib["file"]
    )
    controller_args = {
        item.attrib["name"]: item.attrib["value"]
        for item in controller_include.findall("arg")
    }
    assert controller_args["trajectory_topic"] == "/UAV0/planning/autotrans_trajectory"
    assert controller_args["position_cmd_topic"] == (
        "$(eval '/UAV0/fuel/planning/pos_cmd' if "
        "arg('pure_fuel_mode') else '/UAV0/planning/pos_cmd')"
    )
