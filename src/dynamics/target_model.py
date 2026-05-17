# For a maneuvering target
import numpy as np


def maneuvering_target(initial_pos, velocity, t):
    """
    Maneuvering target with bounded sinusoidal lateral motion.
    Position, velocity, and acceleration are consistent.
    """

    amplitude = 500.0
    omega = 0.08

    pos = initial_pos + velocity * t
    pos[1] += amplitude * np.sin(omega * t)

    vel = velocity.copy()
    vel[1] += amplitude * omega * np.cos(omega * t)

    acc = np.array([
        0.0,
        -amplitude * omega**2 * np.sin(omega * t),
        0.0
    ])

    return pos, vel, acc

# For a constant velocity target
# import numpy as np


# def constant_velocity_target(initial_pos, velocity, t):
#     """
#     Simple target model with constant velocity.
#     """

#     return initial_pos + velocity * t, velocity