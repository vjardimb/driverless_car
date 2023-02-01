import numpy as np
from scipy.integrate import odeint
from scipy.interpolate import interp1d
import cvxpy as cp

import time

import matplotlib.pyplot as plt

from mpc.helpers import get_linear_model, compute_path_from_wp, road_curve, get_nn_idx, f, df, kinematics_model

plt.style.use("ggplot")

# Control problem statement.

N = 3  # number of state variables
M = 2  # number of control variables
T = 20  # Prediction Horizon
dt = 0.25  # discretization step

track = compute_path_from_wp([0, 3, 4, 6, 10, 13], [0, 0, 2, 4, 3, 3], 0.25)

# track = compute_path_from_wp([0,5,7.5,10,12,13,13,10],
#                              [0,0,2.5,2.5,0,0,5,10],0.5)

sim_duration = 80  # time steps
opt_time = []

x_sim = np.zeros((N, sim_duration))
u_sim = np.zeros((M, sim_duration - 1))

MAX_SPEED = 1.25
MIN_SPEED = 0.75
MAX_STEER_SPEED = 1.57

# Starting Condition
x0 = np.zeros(N)
x0[0] = 0
x0[1] = -0.25
x0[2] = np.radians(-0)
x_sim[:, 0] = x0

# starting guess
u_bar = np.zeros((M, T))
u_bar[0, :] = 0.5 * (MAX_SPEED + MIN_SPEED)
u_bar[1, :] = 0.00

for sim_time in range(sim_duration - 1):

    iter_start = time.time()

    K = road_curve(x_sim[:, sim_time], track)

    # dynamics starting state w.r.t vehicle frame
    x_bar = np.zeros((N, T + 1))

    # prediction for linearization of costrains
    for t in range(1, T + 1):
        xt = x_bar[:, t - 1].reshape(N, 1)
        ut = u_bar[:, t - 1].reshape(M, 1)
        A, B, C = get_linear_model(xt, ut, dt, N, M)
        xt_plus_one = np.squeeze(np.dot(A, xt) + np.dot(B, ut) + C)
        x_bar[:, t] = xt_plus_one

    # CVXPY Linear MPC problem statement
    cost = 0
    constr = []
    x = cp.Variable((N, T + 1))
    u = cp.Variable((M, T))

    # Prediction Horizon
    for t in range(T):

        # cost += 30*cp.sum_squares(x[2,t]-np.arctan(df(x_bar[0,t],K))) # psi
        cost += 50 * cp.sum_squares(
            x[2, t] - np.arctan2(df(x_bar[0, t], K), x_bar[0, t])
        )  # psi
        cost += 20 * cp.sum_squares(f(x_bar[0, t], K) - x[1, t])  # cte

        # Actuation rate of change
        if t < (T - 1):
            cost += cp.quad_form(u[:, t + 1] - u[:, t], 100 * np.eye(M))

        # Actuation effort
        cost += cp.quad_form(u[:, t], 1 * np.eye(M))

        # Kinrmatics Constrains (Linearized model)
        A, B, C = get_linear_model(x_bar[:, t], u_bar[:, t], dt, N, M)
        constr += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C.flatten()]

    # sums problem objectives and concatenates constraints.
    constr += [x[:, 0] == x_bar[:, 0]]  # <--watch out the start condition
    constr += [u[0, :] <= MAX_SPEED]
    constr += [u[0, :] >= MIN_SPEED]
    constr += [cp.abs(u[1, :]) <= MAX_STEER_SPEED]

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
    tspan = [0, dt]
    x_sim[:, sim_time + 1] = odeint(
        kinematics_model, x_sim[:, sim_time], tspan, args=(u_bar[:, 0],)
    )[1]

print("CVXPY Optimization Time: Avrg: {:.4f}s Max: {:.4f}s Min: {:.4f}s".format(np.mean(opt_time), np.max(opt_time), np.min(opt_time)))


# plot trajectory
grid = plt.GridSpec(2, 3)

plt.figure(figsize=(15, 10))

plt.subplot(grid[0:2, 0:2])
plt.plot(track[0, :], track[1, :], "b+")
plt.plot(x_sim[0, :], x_sim[1, :])
plt.axis("equal")
plt.ylabel("y")
plt.xlabel("x")

plt.subplot(grid[0, 2])
plt.plot(u_sim[0, :])
plt.ylabel("v(t) [m/s]")

plt.subplot(grid[1, 2])
plt.plot(np.degrees(u_sim[1, :]))
plt.ylabel("w(t) [deg/s]")

plt.tight_layout()
plt.show()