import numpy as np


def solve_discrete_lqr(A, B, Q, R, max_iterations=150, tolerance=1e-8):
    """
    Solve the discrete-time LQR problem using Riccati iteration.
    """

    P = Q.copy()

    for _ in range(max_iterations):
        K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

        P_next = A.T @ P @ A - A.T @ P @ B @ K + Q

        if np.max(np.abs(P_next - P)) < tolerance:
            break

        P = P_next

    K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)

    return K