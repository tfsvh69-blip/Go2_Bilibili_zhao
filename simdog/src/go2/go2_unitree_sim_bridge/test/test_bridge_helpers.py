import math
from pathlib import Path

import pytest

from go2_unitree_sim_bridge.bridge import (
    API_DANCE1,
    API_EULER,
    API_HELLO,
    API_MOVE,
    API_STOP_MOVE,
    API_STRETCH,
    BEHAVIOR_APIS,
    ERROR_BUSY,
    ERROR_DOWNSTREAM,
    ERROR_INVALID_PARAMETER,
    ERROR_UNSUPPORTED_API,
    FOOT_FRAMES,
    MOTOR_JOINTS,
    clamp,
    parse_xyz,
    quaternion_to_rpy,
    rpy_to_quaternion,
)


def test_parse_xyz() -> None:
    assert parse_xyz('{"x": 0.1, "y": -0.2, "z": 0.3}') == (0.1, -0.2, 0.3)


@pytest.mark.parametrize(
    "parameter",
    [
        "",
        "[]",
        '{"x": 1}',
        '{"x": true, "y": 0, "z": 0}',
        '{"x": 1e999, "y": 0, "z": 0}',
    ],
)
def test_parse_xyz_rejects_invalid_input(parameter: str) -> None:
    with pytest.raises(ValueError):
        parse_xyz(parameter)


def test_quaternion_round_trip() -> None:
    expected = (0.2, -0.1, 0.3)
    quaternion = rpy_to_quaternion(*expected)
    actual = quaternion_to_rpy(*quaternion)
    assert actual == pytest.approx(expected, abs=1e-6)
    magnitude = math.sqrt(sum(value * value for value in quaternion))
    assert magnitude == pytest.approx(1.0)


def test_clamp() -> None:
    assert clamp(0.2, 0.3) == 0.2
    assert clamp(0.8, 0.3) == 0.3
    assert clamp(-0.8, 0.3) == -0.3


def test_official_api_ids_and_simulator_errors() -> None:
    assert (API_STOP_MOVE, API_EULER, API_MOVE) == (1003, 1007, 1008)
    assert (API_HELLO, API_STRETCH, API_DANCE1) == (1016, 1017, 1022)
    assert set(BEHAVIOR_APIS) == {
        1004,
        1005,
        1006,
        1009,
        1010,
        1016,
        1017,
        1022,
    }
    assert (
        ERROR_INVALID_PARAMETER,
        ERROR_UNSUPPORTED_API,
        ERROR_BUSY,
        ERROR_DOWNSTREAM,
    ) == (-32001, -32002, -32003, -32004)


def test_unitree_leg_order_is_fr_fl_rr_rl() -> None:
    assert FOOT_FRAMES == [
        "rf_foot_link",
        "lf_foot_link",
        "rh_foot_link",
        "lh_foot_link",
    ]
    assert [name[:2] for name in MOTOR_JOINTS[::3]] == [
        "rf",
        "lf",
        "rh",
        "lh",
    ]


def test_bridge_tolerates_only_shutdown_spin_errors() -> None:
    source = (
        Path(__file__).parents[1]
        / "go2_unitree_sim_bridge"
        / "bridge.py"
    ).read_text(encoding="utf-8")
    assert "except (RuntimeError, TypeError):" in source
    assert "if rclpy.ok():\n            raise" in source
