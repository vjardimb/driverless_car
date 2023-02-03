import numpy as np
from scipy.interpolate import interp1d


def get_linear_model(x_bar, u_bar, DT, N, M):
    """
    Computes the LTI approximated state space model x' = Ax + Bu + C
    """

    L = 0.3  # vehicle wheelbase

    x = x_bar[0]
    y = x_bar[1]
    v = x_bar[2]
    theta = x_bar[3]

    a = u_bar[0]
    delta = u_bar[1]

    A = np.zeros((N, N))
    A[0, 2] = np.cos(theta)
    A[0, 3] = -v * np.sin(theta)
    A[1, 2] = np.sin(theta)
    A[1, 3] = v * np.cos(theta)
    A[3, 2] = v * np.tan(delta) / L
    A_lin = np.eye(N) + DT * A

    B = np.zeros((N, M))
    B[2, 0] = 1
    B[3, 1] = v / (L * np.cos(delta) ** 2)
    B_lin = DT * B

    f_xu = np.array(
        [v * np.cos(theta), v * np.sin(theta), a, v * np.tan(delta) / L]
    ).reshape(N, 1)
    C_lin = DT * (
        f_xu - np.dot(A, x_bar.reshape(N, 1)) - np.dot(B, u_bar.reshape(M, 1))
    )

    return np.round(A_lin, 4), np.round(B_lin, 4), np.round(C_lin, 4)


def compute_path_from_wp(start_xp, start_yp, step=0.1):
    """
    Computes a reference path given a set of waypoints
    """

    final_xp = []
    final_yp = []
    delta = step  # [m]

    for idx in range(len(start_xp) - 1):
        section_len = np.sum(
            np.sqrt(
                np.power(np.diff(start_xp[idx : idx + 2]), 2)
                + np.power(np.diff(start_yp[idx : idx + 2]), 2)
            )
        )

        interp_range = np.linspace(0, 1, np.floor(section_len / delta).astype(int))

        fx = interp1d(np.linspace(0, 1, 2), start_xp[idx : idx + 2], kind=1)
        fy = interp1d(np.linspace(0, 1, 2), start_yp[idx : idx + 2], kind=1)

        final_xp = np.append(final_xp, fx(interp_range))
        final_yp = np.append(final_yp, fy(interp_range))

    dx = np.append(0, np.diff(final_xp))
    dy = np.append(0, np.diff(final_yp))
    theta = np.arctan2(dy, dx)

    return np.vstack((final_xp, final_yp, theta))


def road_curve(state, track):

    # given vehicle pos find lookahead waypoints
    nn_idx = get_nn_idx(state, track) - 1
    LOOKAHED = 6
    lk_wp = track[:, nn_idx : nn_idx + LOOKAHED]

    # trasform lookahead waypoints to vehicle ref frame
    dx = lk_wp[0, :] - state[0]
    dy = lk_wp[1, :] - state[1]

    wp_vehicle_frame = np.vstack(
        (
            dx * np.cos(-state[2]) - dy * np.sin(-state[2]),
            dy * np.cos(-state[2]) + dx * np.sin(-state[2]),
        )
    )

    # fit poly
    return np.polyfit(
        wp_vehicle_frame[0, :],
        wp_vehicle_frame[1, :],
        3,
        rcond=None,
        full=False,
        w=None,
        cov=False,
    )


def get_nn_idx(state, path):
    """
    Computes the index of the waypoint closest to vehicle
    """

    dx = state[0] - path[0, :]
    dy = state[1] - path[1, :]
    dist = np.hypot(dx, dy)
    nn_idx = np.argmin(dist)

    try:
        v = [
            path[0, nn_idx + 1] - path[0, nn_idx],
            path[1, nn_idx + 1] - path[1, nn_idx],
        ]
        v /= np.linalg.norm(v)

        d = [path[0, nn_idx] - state[0], path[1, nn_idx] - state[1]]

        if np.dot(d, v) > 0:
            target_idx = nn_idx
        else:
            target_idx = nn_idx + 1

    except IndexError as e:
        target_idx = nn_idx

    return target_idx


def f(x, coeff):
    return round(
        coeff[0] * x**3 + coeff[1] * x**2 + coeff[2] * x**1 + coeff[3] * x**0, 6
    )


def df(x, coeff):
    return round(3 * coeff[0] * x**2 + 2 * coeff[1] * x**1 + coeff[2] * x**0, 6)


# Define process model
# This uses the continuous model
def kinematics_model(x, t, u):
    """
    Returns the set of ODE of the vehicle model.
    """

    L = 0.3  # vehicle wheelbase
    dxdt = x[2] * np.cos(x[3])
    dydt = x[2] * np.sin(x[3])
    dvdt = u[0]
    dthetadt = x[2] * np.tan(u[1]) / L

    dqdt = [dxdt, dydt, dvdt, dthetadt]

    return dqdt

def get_ref_trajectory(state, path, target_v, N, T, DT):
    """
    Adapted from pythonrobotics
    """
    xref = np.zeros((N, T + 1))
    dref = np.zeros((1, T + 1))

    # sp = np.ones((1,T +1))*target_v #speed profile

    ncourse = path.shape[1]

    ind = get_nn_idx(state, path)

    xref[0, 0] = path[0, ind]  # X
    xref[1, 0] = path[1, ind]  # Y
    xref[2, 0] = target_v  # sp[ind] #V
    xref[3, 0] = path[2, ind]  # Theta
    dref[0, 0] = 0.0  # steer operational point should be 0

    dl = 0.05  # Waypoints spacing [m]
    travel = 0.0

    for i in range(T + 1):
        travel += abs(target_v) * DT  # current V or target V?
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = path[0, ind + dind]
            xref[1, i] = path[1, ind + dind]
            xref[2, i] = target_v  # sp[ind + dind]
            xref[3, i] = path[2, ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = path[0, ncourse - 1]
            xref[1, i] = path[1, ncourse - 1]
            xref[2, i] = 0.0  # stop? #sp[ncourse - 1]
            xref[3, i] = path[2, ncourse - 1]
            dref[0, i] = 0.0

    return xref, dref