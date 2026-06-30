# Copyright (c) 2021, Matthew Booker
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. Reserved.

import numpy as np


def normalize_angle(angle):
    """
    Normalize the angle to between [0, 2pi).

    Args:
    angle: float
        The angle to normalize in radians

    Returns
    -------
    The normalized angle in the range [0,2pi)

    """
    while angle >= 2*np.pi:
        angle -= 2*np.pi

    while angle < 0:
        angle += 2*np.pi

    return angle


def angle_difference(angle_1, angle_2, left_turn=None):
    """
    Calculate the difference between two angles based on a given direction.

    Args:
    angle_1: float
        The starting angle in radians
    angle_2: float
        The ending angle in radians
    left_turn: bool
        The direction of turn. True if left, false if right
        and None if smallest angular difference should be
        returned

    Returns
    -------
    The angular difference between the two angles according to
    the specified turn direction

    """
    if left_turn is None:
        dif = abs(angle_1 - angle_2)

        return dif if dif <= np.pi else 2 * np.pi - dif

    elif left_turn:

        if angle_2 >= angle_1:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)

    else:
        if angle_1 >= angle_2:
            return abs(angle_1 - angle_2)
        else:
            return 2 * np.pi - abs(angle_1 - angle_2)


def interpolate_yaws(start_angle, end_angle, left_turn, steps):
    """
    Create equally spaced yaws between two angles.

    Args:
    start_angle: float
        The starting angle
    end_angle: float
        The ending angle
    left_turn: bool
        The direction of turn. True if left, False otherwise
    steps: int
        The number of yaws to generate between start and end
        angle

    Returns
    -------
    An array of yaws starting at start angle and ending at end
    angle with steps number of angles between them

    """
    if left_turn:
        if start_angle > end_angle:
            end_angle += 2 * np.pi
    else:
        if end_angle > start_angle:
            end_angle -= 2 * np.pi

    yaws = np.linspace(start_angle, end_angle, steps)
    yaws = np.vectorize(normalize_angle)(yaws)

    return yaws


def get_rotation_matrix(angle):
    """
    Return a rotation matrix that is equivalent to a 2D rotation of angle.

    Args:
    angle: float
        The angle to create a rotation matrix for

    Returns
    -------
    A 2x2 matrix representing a 2D rotation by angle

    """
    return np.array([[np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)]])


def compute_curvature_samples(xs, ys):
    """
    Compute signed curvature samples aligned with input pose samples.

    Curvature is approximated as d(theta)/ds over consecutive samples.

    Args:
    xs, ys: array-like
        Sampled trajectory state arrays of equal length.

    Returns
    -------
    np.ndarray
        Curvature values (1/m), same length as input arrays.

    """
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)

    if xs.size != ys.size:
        raise ValueError('xs and ys must have identical lengths')

    if xs.size == 0:
        return np.array([], dtype=float)

    if xs.size == 1:
        return np.array([0.0], dtype=float)

    # Segement legth
    dx = np.diff(xs)
    dy = np.diff(ys)
    ds = np.hypot(dx, dy)
    curve_pos = np.cumsum(ds)   # Cumulative distance along the curve
    curve_pos = np.insert(curve_pos, 0, 0.0)  # Add starting position
    # deltas = np.vstack((dx, dy)).T

    # Filter out segments that are too short to compute curvature
    valid_steps = ds > ds.mean()/2
    valid_steps = np.insert(valid_steps, 0, True)  
    xs_select = xs[valid_steps]
    ys_select = ys[valid_steps]
    
    dx_select = np.diff(xs_select)
    dy_select = np.diff(ys_select)
    ds_select = np.hypot(dx_select, dy_select)
    
    # Wrapped heading difference in [-pi, pi]
    path_direction = np.arctan2(dy_select, dx_select)
    dtheta = np.arctan2(np.sin(np.diff(path_direction)), np.cos(np.diff(path_direction)))
    # dtheta = np.arctan2(np.cross(deltas[:-1], deltas[1:]), np.dot(deltas[:-1], deltas[1:].T).diagonal())

    # Saturate dtheta to pi/2
    dtheta = np.clip(dtheta, -np.pi/2, np.pi/2)

    # Compute curvature as dtheta/ds
    # Use one-sided difference for the endpoints and central difference for the interior points
    machine_epsilon = 1e-9
    kappas_select = np.zeros_like(xs_select)
    kappas_select[0] = dtheta[0] / (ds_select[0] + machine_epsilon)  # Avoid division by zero
    kappas_select[-1] = dtheta[-1] / (ds_select[-1] + machine_epsilon)  # Avoid division by zero
    kappas_select[1:-1] = 2 * dtheta / (ds_select[:-1] + ds_select[1:] + machine_epsilon)  # Avoid division by zero

    # Overwrite when ds is zero
    non_valid = ds_select < machine_epsilon
    non_valid = np.insert(non_valid, 0, non_valid[0])  # First point has no previous segment
    # if any(non_valid):
        # print(f"Warning: Zero segment length detected at indices {np.where(non_valid)[0]}. Setting curvature to 0.")    
    kappas_select[non_valid] = 0.0
    
    # Resample curvature to match the original number of samples using linear interpolation    
    # if not np.all(valid_steps):
    #     print(f"Warning: Some segments were too short to compute curvature. Indices: {np.where(~valid_steps)[0]}")
    kappas = np.interp(curve_pos, curve_pos[valid_steps], kappas_select)

    return kappas

