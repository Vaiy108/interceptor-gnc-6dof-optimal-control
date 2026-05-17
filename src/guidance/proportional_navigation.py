import numpy as np


def proportional_navigation(interceptor_pos, interceptor_vel, target_pos, target_vel, nav_constant=3.0):
    """
    3D Proportional Navigation guidance law.

    Returns commanded acceleration vector.
    """

    r = target_pos - interceptor_pos
    v_rel = target_vel - interceptor_vel

    r_norm = np.linalg.norm(r)

    if r_norm < 1e-6:
        return np.zeros(3)

    los_unit = r / r_norm

    closing_velocity = -np.dot(v_rel, los_unit)

    los_rate = np.cross(r, v_rel) / (r_norm ** 2)

    a_cmd = nav_constant * closing_velocity * np.cross(los_rate, los_unit)

    return a_cmd