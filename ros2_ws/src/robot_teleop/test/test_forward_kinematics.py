import math

import pytest

from robot_teleop.forward_kinematics import end_effector_matrix


def test_zero_angles_match_original_dh_chain():
    matrix = end_effector_matrix([0.0, 0.0, 0.0])

    expected = [
        [1.0, 0.0, 0.0, 4.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for actual_row, expected_row in zip(matrix, expected):
        assert actual_row == pytest.approx(expected_row)


def test_result_is_homogeneous_for_nonzero_angles():
    matrix = end_effector_matrix([25.0, -30.0, 45.0])

    assert matrix[3] == pytest.approx([0.0, 0.0, 0.0, 1.0])
    for row in range(3):
        norm = math.sqrt(sum(matrix[index][row] ** 2 for index in range(3)))
        assert norm == pytest.approx(1.0)


def test_requires_three_angles():
    with pytest.raises(ValueError):
        end_effector_matrix([0.0, 0.0])
