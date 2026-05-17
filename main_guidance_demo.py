import numpy as np
import matplotlib.pyplot as plt

from src.guidance.proportional_navigation import proportional_navigation
#from src.dynamics.target_model import constant_velocity_target
from src.dynamics.target_model import maneuvering_target

from src.simulation.integrator import integrate_point_mass
from src.dynamics.interceptor_6dof import Interceptor6DOF

from src.guidance.augmented_pn import (
    augmented_proportional_navigation
)

def main():
    dt = 0.01
    sim_time = 60.0
    steps = int(sim_time / dt)

    nav_constant = 3.0
    max_acc = 120.0

    # interceptor_pos = np.array([0.0, 0.0, 0.0])
    # interceptor_vel = np.array([300.0, 0.0, 0.0]) 

    interceptor = Interceptor6DOF()

    target_initial_pos = np.array([5000.0, 1000.0, 500.0])
    target_vel = np.array([180.0, -20.0, 0.0])

    interceptor_history = []
    target_history = []
    miss_distance_history = []

    min_miss_distance = float("inf")
    best_time = 0.0
    previous_miss_distance = float("inf")

    for k in range(steps):
        t = k * dt

        # target_pos, target_current_vel = constant_velocity_target(
        #     target_initial_pos,
        #     target_vel,
        #     t
        # )
        # position, velocity from maneuvering target
        target_pos, target_current_vel, target_acc = maneuvering_target(
            target_initial_pos,
            target_vel,
            t
        )

        state = interceptor.get_state()

        interceptor_pos = state["position"]
        interceptor_vel = state["velocity"]

        a_cmd = augmented_proportional_navigation(
            interceptor_pos,
            interceptor_vel,
            target_pos,
            target_current_vel,
            target_acc,
            nav_constant=3.0,
            target_acc_gain=0.5
        )
        # Interceptor for point-mass object
        # interceptor_pos, interceptor_vel = integrate_point_mass(
        #     interceptor_pos,
        #     interceptor_vel,
        #     a_cmd,
        #     dt,
        #     max_acc=max_acc
        # )
        # Interceptor for object using rigid-body dynamics
        interceptor.step(a_cmd, dt)

        state = interceptor.get_state()

        interceptor_pos = state["position"]
        interceptor_vel = state["velocity"]

        miss_distance = np.linalg.norm(target_pos - interceptor_pos)

        interceptor_history.append(interceptor_pos.copy())
        target_history.append(target_pos.copy())
        miss_distance_history.append(miss_distance)

        if miss_distance < min_miss_distance:
            min_miss_distance = miss_distance
            best_time = t

        if miss_distance < 10.0:
            print(f"Intercept achieved at t = {t:.2f} s")
            break

        if miss_distance > previous_miss_distance and previous_miss_distance < 50.0:
            print(f"Closest approach reached at t = {best_time:.2f} s")
            break

        previous_miss_distance = miss_distance

    interceptor_history = np.array(interceptor_history)
    target_history = np.array(target_history)

    print(f"Minimum miss distance: {min_miss_distance:.2f} m")
    print(f"Closest approach time: {best_time:.2f} s")

    fig = plt.figure(figsize=(12, 8))

    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        interceptor_history[:, 0],
        interceptor_history[:, 1],
        interceptor_history[:, 2],
        linewidth=2.5,
        label="Interceptor"
    )

    ax.plot(
        target_history[:, 0],
        target_history[:, 1],
        target_history[:, 2],
        linewidth=2.5,
        label="Target"
    )

    ax.set_xlabel("X Position [m]", fontsize=14, labelpad=15)
    ax.set_ylabel("Y Position [m]", fontsize=14, labelpad=15)
    ax.set_zlabel("Z Position [m]", fontsize=14, labelpad=15)

    ax.set_title(
        "6-DOF Augmented PN Intercept",
        fontsize=24,
        pad=25
    )

    ax.legend(fontsize=14)

    ax.tick_params(axis='both', which='major', labelsize=11)

    ax.view_init(elev=24, azim=-58)

    plt.tight_layout()

    plt.savefig(
        "results/apn_maneuvering_target_intercept.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()

if __name__ == "__main__":
    main()