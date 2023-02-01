import numpy as np
from scipy.interpolate import interp1d


def get_linear_model(x_bar, u_bar, dt, N, M):
    """ """

    x = x_bar[0]
    y = x_bar[1]
    theta = x_bar[2]

    v = u_bar[0]
    w = u_bar[1]

    A = np.zeros((N, N))
    A[0, 2] = -v * np.sin(theta)
    A[1, 2] = v * np.cos(theta)
    A_lin = np.eye(N) + dt * A

    B = np.zeros((N, M))
    B[0, 0] = np.cos(theta)
    B[1, 0] = np.sin(theta)
    B[2, 1] = 1
    B_lin = dt * B

    f_xu = np.array([v * np.cos(theta), v * np.sin(theta), w]).reshape(N, 1)
    C_lin = dt * (
        f_xu - np.dot(A, x_bar.reshape(N, 1)) - np.dot(B, u_bar.reshape(M, 1))
    )

    return A_lin, B_lin, C_lin


def compute_path_from_wp(start_xp, start_yp, step=0.1):
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

    return np.vstack((final_xp, final_yp))


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

    dx = state[0] - path[0, :]
    dy = state[1] - path[1, :]
    dist = np.sqrt(dx**2 + dy**2)
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
    """ """

    dxdt = u[0] * np.cos(x[2])
    dydt = u[0] * np.sin(x[2])
    dthetadt = u[1]

    dqdt = [dxdt, dydt, dthetadt]

    return dqdt