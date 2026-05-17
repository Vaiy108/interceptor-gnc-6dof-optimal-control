import numpy as np


def integrate_point_mass(pos, vel, acc, dt, max_acc=None):
    """
    Simple point-mass integration with optional acceleration limit.
    """

    if max_acc is not None:
        acc_norm = np.linalg.norm(acc)

        if acc_norm > max_acc:
            acc = acc / acc_norm * max_acc

    vel_next = vel + acc * dt
    pos_next = pos + vel_next * dt

    return pos_next, vel_next