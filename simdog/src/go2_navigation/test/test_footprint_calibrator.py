"""Go2 Footprint collision 投影与统计余量测试。"""

import math

import numpy as np
import pytest

from go2_navigation.footprint_calibrator import (
    CollisionShape,
    calibration_statistics,
    convex_hull,
    footprint_string,
    pad_footprint,
    parse_collision_shapes,
    parse_footprint_parameter,
    polygon_area,
    project_shape,
    transform_planar_points,
)


SAMPLE_URDF = """
<robot name="sample">
  <link name="body">
    <collision name="body_box">
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <geometry><box size="1.0 0.4 0.2"/></geometry>
    </collision>
  </link>
  <link name="foot">
    <collision><geometry><sphere radius="0.02"/></geometry></collision>
  </link>
  <link name="sensor">
    <collision>
      <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
      <geometry><cylinder radius="0.05" length="0.06"/></geometry>
    </collision>
  </link>
</robot>
"""


def test_parse_collision_shapes_keeps_sources_origins_and_dimensions():
    shapes = parse_collision_shapes(SAMPLE_URDF)
    assert [(shape.link, shape.kind) for shape in shapes] == [
        ("body", "box"),
        ("foot", "sphere"),
        ("sensor", "cylinder"),
    ]
    assert shapes[0].source == "body:body_box"
    assert shapes[0].origin_xyz == pytest.approx((0.1, 0.0, 0.0))
    assert shapes[2].dimensions == pytest.approx((0.05, 0.06))


def test_parse_collision_shapes_refuses_mesh_instead_of_silently_ignoring_it():
    urdf = """
    <robot name="unsafe"><link name="body"><collision><geometry>
      <mesh filename="body.stl"/>
    </geometry></collision></link></robot>
    """
    with pytest.raises(ValueError, match="mesh"):
        parse_collision_shapes(urdf)


def test_project_box_applies_collision_origin_and_link_rotation():
    shape = CollisionShape(
        "body:0",
        "body",
        "box",
        (0.2, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (2.0, 1.0, 0.4),
    )
    half = math.sqrt(0.5)
    points = project_shape(shape, (1.0, 2.0, 0.0), (0.0, 0.0, half, half))
    # link 旋转 90° 后，collision origin 沿 +y 偏移 0.2 m。
    assert np.min(points[:, 0]) == pytest.approx(0.5)
    assert np.max(points[:, 0]) == pytest.approx(1.5)
    assert np.min(points[:, 1]) == pytest.approx(1.2)
    assert np.max(points[:, 1]) == pytest.approx(3.2)


def test_convex_hull_removes_interior_points_and_is_counterclockwise():
    hull = convex_hull(
        [(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (1, 0)]
    )
    assert hull == [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    assert polygon_area(hull) == pytest.approx(1.0)


def test_padding_is_statistical_tail_plus_half_costmap_cell_rounded_up():
    normal = np.asarray([[-0.2, -0.1], [0.2, -0.1], [0.2, 0.1], [-0.2, 0.1]])
    samples = [normal.copy() for _ in range(100)]
    samples[-1] = np.asarray(
        [[-0.2, -0.1], [0.22, -0.1], [0.22, 0.1], [-0.2, 0.1]]
    )
    result = calibration_statistics(samples, resolution=0.05)
    assert result["half_costmap_cell_m"] == pytest.approx(0.025)
    assert result["statistical_tail_m"] > 0.0
    assert result["recommended_padding_m"] >= (
        result["half_costmap_cell_m"] + result["statistical_tail_m"]
    )
    padding_steps = result["recommended_padding_m"] / 0.005
    assert padding_steps == pytest.approx(round(padding_steps))


def test_footprint_string_round_trips_without_closing_duplicate():
    text = footprint_string(
        [(-0.1236, -0.2), (0.3, -0.2), (0.3001, -0.2001), (0.3, 0.2)]
    )
    assert text == "[[-0.124,-0.2],[0.3,-0.2],[0.3,0.2]]"
    assert parse_footprint_parameter(text) == [
        (-0.124, -0.2),
        (0.3, -0.2),
        (0.3, 0.2),
    ]


def test_pad_footprint_matches_nav2_sign_zero_axis_expansion():
    assert np.allclose(
        pad_footprint([(-0.4, -0.1), (0.3, 0.0)], 0.035),
        [(-0.435, -0.135), (0.335, 0.0)],
    )
    with pytest.raises(ValueError, match="非负有限数"):
        pad_footprint([(0.0, 0.0)], -0.01)


def test_transform_planar_points_applies_runtime_tf_back_to_base_frame():
    half = math.sqrt(0.5)
    points = transform_planar_points(
        [(1.0, 0.0), (0.0, 1.0)],
        (2.0, 3.0, 0.0),
        (0.0, 0.0, half, half),
    )
    assert points == pytest.approx([(2.0, 4.0), (1.0, 3.0)])
