import numpy as np


def augmented_proportional_navigation(
    interceptor_pos,
    interceptor_vel,
    target_pos,
    target_vel,
    target_acc,
    nav_constant=3.0,
    target_acc_gain=1.0
):
    """
    Augmented Proportional Navigation (APN).
    """

    r = target_pos - interceptor_pos

    v_rel = target_vel - interceptor_vel

    r_norm = np.linalg.norm(r)

    if r_norm < 1e-6:
        return np.zeros(3)

    los_unit = r / r_norm

    closing_velocity = -np.dot(v_rel, los_unit)

    los_rate = np.cross(r, v_rel) / (r_norm ** 2)

    pn_term = (
        nav_constant *
        closing_velocity *
        np.cross(los_rate, los_unit)
    )

    apn_term = target_acc_gain * target_acc

    return pn_term + apn_term