import numpy as np
import matplotlib.pyplot as plt

from src.control.lqr_controller import solve_discrete_lqr


def main():
    dt = 0.05
    sim_time = 35.0
    steps = int(sim_time / dt)

    # State: [x, y, vx, vy]
    A = np.array([
        [1.0, 0.0, dt,  0.0],
        [0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])

    # Input: [ax, ay]
    B = np.array([
        [0.5 * dt**2, 0.0],
        [0.0, 0.5 * dt**2],
        [dt, 0.0],
        [0.0, dt]
    ])

    # Penalize position error strongly and velocity error moderately
    #Q = np.diag([10.0, 10.0, 1.0, 1.0])
    # increases position correction, damps velocity, reduce overshoot
    Q = np.diag([40.0, 40.0, 12.0, 12.0])

    # Penalize control effort
    #R = np.diag([0.1, 0.1])
    # to prevent agressive acc, momentum
    R = np.diag([0.3, 0.3])

    K = solve_discrete_lqr(A, B, Q, R)

    state = np.array([0.0, 0.0, 0.0, 0.0])

    target_state = np.array([1000.0, 500.0, 0.0, 0.0])

    max_acc = 60.0

    state_history = []
    control_history = []
    time_history = []

    for k in range(steps):
        t = k * dt

        error = state - target_state

        u = -K @ error

        acc_norm = np.linalg.norm(u)

        if acc_norm > max_acc:
            u = u / acc_norm * max_acc

        state = A @ state + B @ u

        state_history.append(state.copy())
        control_history.append(u.copy())
        time_history.append(t)

        position_error = np.linalg.norm(state[:2] - target_state[:2])

        if position_error < 1.0 and np.linalg.norm(state[2:]) < 0.5:
            print(f"Target reached at t = {t:.2f} s")
            break

    state_history = np.array(state_history)
    control_history = np.array(control_history)
    time_history = np.array(time_history)

    final_position_error = np.linalg.norm(state_history[-1, :2] - target_state[:2])

    print("LQR Optimal-Control Results")
    print("---------------------------")
    print(f"Final position error: {final_position_error:.2f} m")
    print(f"Final velocity: {np.linalg.norm(state_history[-1, 2:]):.2f} m/s")
    print(f"Maximum commanded acceleration: {np.max(np.linalg.norm(control_history, axis=1)):.2f} m/s²")

    plt.figure(figsize=(10, 7))

    plt.plot(
        state_history[:, 0],
        state_history[:, 1],
        linewidth=2.5,
        label="LQR trajectory"
    )

    plt.scatter(
        target_state[0],
        target_state[1],
        s=100,
        marker="x",
        label="Target point"
    )

    plt.xlabel("X Position [m]", fontsize=13)
    plt.ylabel("Y Position [m]", fontsize=13)
    plt.title("LQR Optimal-Control Trajectory Shaping", fontsize=18)
    plt.legend(fontsize=12)
    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        "results/optimal_control_lqr_trajectory.png",
        dpi=300,
        bbox_inches="tight"
    )

    plt.show()


if __name__ == "__main__":
    main()