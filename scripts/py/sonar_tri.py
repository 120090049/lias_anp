import numpy as np

def sonar_triangulation(T1, T2, d1, theta1, d2, theta2):
    """
    T1: The first sonar pose w.r.t. the world frame, represented as a 4x4 matrix [R1, t1; 0, 1].
    T2: The second sonar pose w.r.t. the world frame, represented as a 4x4 matrix [R2, t2; 0, 1].
    d1: The distance measurement of the first sonar in meters.
    theta1: The angle measurement of the first sonar in radians.
    d2: The distance measurement of the second sonar in meters.
    theta2: The angle measurement of the second sonar in radians.
    Returns:
    x: The estimated coordinates of the 3D point in the first sonar frame.
    """
    # Calculate the relative transformation
    T = np.linalg.inv(T2) @ T1
    R = T[:3, :3]
    t = T[:3, 3]

    r1 = R[0, :]
    r2 = R[1, :]

    # Solve the first set of linear equations
    A1 = np.vstack([
        [np.tan(theta1), -1, 0],
        [np.tan(theta2) * r1 - r2],
        [t @ R]
    ])
    b1 = np.array([0, t[1] - np.tan(theta2) * t[0], (d2**2 - d1**2 - np.linalg.norm(t)**2) / 2])

    x = np.linalg.inv(A1) @ b1

    # Solve the second set of linear equations
    A2 = np.vstack([A1, x])
    b2 = np.append(b1, d1**2)

    x = np.linalg.inv(A2.T @ A2) @ A2.T @ b2

    return x
