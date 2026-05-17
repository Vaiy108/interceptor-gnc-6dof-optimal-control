import numpy as np
import matplotlib.pyplot as plt

from src.dynamics.interceptor_6dof import Interceptor6DOF
from src.dynamics.target_model import maneuvering_target
from src.guidance.augmented_pn import augmented_proportional_navigation


def run_single_engagement(target_initial_pos, target_vel, nav_constant=3.0):
    dt = 0.01
    sim_time = 60.0
    steps = int(sim_time / dt)

    interceptor = Interceptor6DOF()

    min_miss_distance = float("inf")
    best_time = 0.0
    previous_miss_distance = float("inf")

    for k in range(steps):
        t = k * dt

        state = interceptor.get_state()

        interceptor_pos = state["position"]
        interceptor_vel = state["velocity"]

        target_pos, target_current_vel, target_acc = maneuvering_target(
            target_initial_pos,
            target_vel,
            t
        )

        a_cmd = augmented_proportional_navigation(
            interceptor_pos,
            interceptor_vel,
            target_pos,
            target_current_vel,
            target_acc,
            nav_constant=nav_constant,
            target_acc_gain=0.5
        )

        interceptor.step(a_cmd, dt)

        state = interceptor.get_state()

        interceptor_pos = state["position"]

        miss_distance = np.linalg.norm(target_pos - interceptor_pos)

        if miss_distance < min_miss_distance:
            min_miss_distance = miss_distance
            best_time = t

        if miss_distance < 10.0:
            break

        if miss_distance > previous_miss_distance and previous_miss_distance < 50.0:
            break

        previous_miss_distance = miss_distance

    return min_miss_distance, best_time


def main():
    np.random.seed(7)

    num_runs = 100

    miss_distances = []
    intercept_times = []

    for _ in range(num_runs):
        target_initial_pos = np.array([
            5000.0 + np.random.uniform(-500.0, 500.0),
            1000.0 + np.random.uniform(-300.0, 300.0),
            500.0 + np.random.uniform(-100.0, 100.0)
        ])

        target_vel = np.array([
            180.0 + np.random.uniform(-20.0, 20.0),
            -20.0 + np.random.uniform(-10.0, 10.0),
            0.0
        ])

        miss_distance, intercept_time = run_single_engagement(
            target_initial_pos,
            target_vel
        )

        miss_distances.append(miss_distance)
        intercept_times.append(intercept_time)

    miss_distances = np.array(miss_distances)
    intercept_times = np.array(intercept_times)

    print("Monte Carlo Results")
    print("-------------------")
    print(f"Runs: {num_runs}")
    print(f"Mean miss distance: {np.mean(miss_distances):.2f} m")
    print(f"Median miss distance: {np.median(miss_distances):.2f} m")
    print(f"Max miss distance: {np.max(miss_distances):.2f} m")
    print(f"Successful intercepts (<10 m): {np.sum(miss_distances < 10.0)} / {num_runs}")

    plt.figure(figsize=(10, 6))

    plt.hist(miss_distances, bins=20, edgecolor="black")

    plt.xlabel("Minimum Miss Distance [m]", fontsize=13)
    plt.ylabel("Number of Runs", fontsize=13)
    plt.title("Monte Carlo Miss Distance Distribution", fontsize=18)

    plt.tight_layout()

    plt.savefig(
        "results/monte_carlo_miss_distance.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


if __name__ == "__main__":
    main()