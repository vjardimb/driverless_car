import numpy as np
from scipy.integrate import odeint
from scipy.interpolate import interp1d
import cvxpy as cp

import time

import matplotlib.pyplot as plt

from mpc.helpers import (
    get_linear_model,
    compute_path_from_wp,
    kinematics_model,
    get_ref_trajectory,
    plot_results
)

plt.style.use("ggplot")

# Control problem statement.

N = 4  # number of state variables
M = 2  # number of control variables
T = 20  # Prediction Horizon (steps)
DT = 0.2  # discretization step

track_points = (
    [0, 3, 4, 6, 10, 12, 14, 6, 1, 0],
    [0, 0, 2, 4, 3, 3, -2, -6, -2, -2]
)

track_points_2 = (
    [0, 10, 10, 0],
    [0, 0, 1, 1]
)

dl = 0.2  # Waypoints spacing [m]

track = compute_path_from_wp(track_points[0], track_points[1], dl)

sim_duration = 200  # time steps
opt_time = []

x_sim = np.zeros((N, sim_duration))
u_sim = np.zeros((M, sim_duration - 1))

MAX_SPEED = 1.5  # m/s
MAX_ACC = 1.0  # m/ss
MAX_D_ACC = 1.0  # m/sss
MAX_STEER = np.radians(30)  # rad
MAX_D_STEER = np.radians(30)  # rad/s

REF_VEL = 1.0  # m/s

# Starting Condition
x0 = np.zeros(N)
x0[0] = 0  # x
x0[1] = -0.25  # y
x0[2] = 0.0  # v
x0[3] = np.radians(-0)  # yaw

# starting guess
u_bar = np.zeros((M, T))
u_bar[0, :] = MAX_ACC / 2  # a
u_bar[1, :] = 0.0  # delta

for sim_time in range(sim_duration - 1):

    iter_start = time.time()

    # dynamics starting state
    x_bar = np.zeros((N, T + 1))
    x_bar[:, 0] = x_sim[:, sim_time]

    # prediction for linearization of constrains
    for t in range(1, T + 1):
        xt = x_bar[:, t - 1].reshape(N, 1)
        ut = u_bar[:, t - 1].reshape(M, 1)
        A, B, C = get_linear_model(xt, ut, DT, N, M)
        xt_plus_one = np.squeeze(np.dot(A, xt) + np.dot(B, ut) + C)
        x_bar[:, t] = xt_plus_one

    # CVXPY Linear MPC problem statement
    x = cp.Variable((N, T + 1))
    u = cp.Variable((M, T))
    cost = 0
    constr = []

    # Cost Matrices
    Q = np.diag([20, 20, 10, 0])  # state error cost
    Qf = np.diag([30, 30, 30, 0])  # state final error cost
    R = np.diag([10, 10])  # input cost
    R_ = np.diag([10, 10])  # input rate of change cost

    # Get reference trajectory
    x_ref, d_ref = get_ref_trajectory(x_bar[:, 0], track, REF_VEL, N, T, DT, dl)

    # Prediction Horizon
    for t in range(T):

        # Tracking Error
        cost += cp.quad_form(x[:, t] - x_ref[:, t], Q)

        # Actuation effort
        cost += cp.quad_form(u[:, t], R)

        # Actuation rate of change
        if t < (T - 1):
            cost += cp.quad_form(u[:, t + 1] - u[:, t], R_)

            constr += [cp.abs(u[0, t + 1] - u[0, t]) / DT <= MAX_D_ACC]  # max acc rate of change
            constr += [cp.abs(u[1, t + 1] - u[1, t]) / DT <= MAX_D_STEER]  # max steer rate of change

        # Kinematics Constraints (Linearized model)
        A, B, C = get_linear_model(x_bar[:, t], u_bar[:, t], DT, N, M)
        constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C.flatten()]

    # Final Point tracking
    cost += cp.quad_form(x[:, T] - x_ref[:, T], Qf)

    # sums problem objectives and concatenates constraints.
    constr += [x[:, 0] == x_bar[:, 0]]  # starting condition
    constr += [x[2, :] <= MAX_SPEED]  # max speed
    constr += [x[2, :] >= 0.0]  # min_speed (not really needed)
    constr += [cp.abs(u[0, :]) <= MAX_ACC]  # max acc
    constr += [cp.abs(u[1, :]) <= MAX_STEER]  # max steer

    # Solve
    prob = cp.Problem(cp.Minimize(cost), constr)
    solution = prob.solve(solver=cp.OSQP, verbose=False)

    # retrieved optimized U and assign to u_bar to linearize in next step
    u_bar = np.vstack(
        (np.array(u.value[0, :]).flatten(), (np.array(u.value[1, :]).flatten()))
    )

    u_sim[:, sim_time] = u_bar[:, 0]

    # Measure elpased time to get results from cvxpy
    opt_time.append(time.time() - iter_start)

    # move simulation to t+1
    tspan = [0, DT]
    x_sim[:, sim_time + 1] = odeint(kinematics_model, x_sim[:, sim_time], tspan, args=(u_bar[:, 0],))[1]

print(
    "CVXPY Optimization Time: Avrg: {:.4f}s Max: {:.4f}s Min: {:.4f}s".format(
        np.mean(opt_time), np.max(opt_time), np.min(opt_time)
    )
)

plot_results(x_sim, u_sim, track)

