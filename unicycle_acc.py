from rockit import *
from casadi import *

import numpy as np


nx = 5
nu = 2
Nhor = 20

# Regularisation used inside the geometric-curvature expression k = w / sqrt(v^2 + eps^2)
# to keep it finite (and differentiable) when v ~ 0.
CURVATURE_EPS = 1e-3

starting_angle = 0.0
start_x = 0.0
start_y = 0.0
start_v = 0.0
start_w = 0.0

end_angle = 0.0
end_x = 1.0
end_y = 0.0

current_X = vertcat(start_x,start_y,starting_angle,start_v,start_w)

T0 = 0.0

ocp = Ocp(T=FreeTime(T0))

x = ocp.state()
y = ocp.state()
theta = ocp.state()
v = ocp.state()
w = ocp.state()

v_dot = ocp.control()
w_dot = ocp.control()

X_0 = ocp.parameter(nx)
final_x = ocp.parameter()
final_y = ocp.parameter()
final_angle = ocp.parameter()
upper_speed = ocp.parameter()
lower_speed = ocp.parameter()
max_rot_speed = ocp.parameter()
max_linear_acc = ocp.parameter()
max_rot_acc = ocp.parameter()
kappa_start = ocp.parameter()
kappa_end = ocp.parameter()
curvature_weight = ocp.parameter()
# Soft weight for v(0) = w(0) = 0. Set high (e.g. 1e6) to reproduce the
# original hard-fixed initial state; set to 0 when the curvature soft
# constraint is active so the OCP can pick a non-zero start velocity.
initial_state_slack_weight = ocp.parameter()

ocp.set_der(x, (v*cos(theta)))
ocp.set_der(y, (v*sin(theta)))
ocp.set_der(theta, w)
ocp.set_der(v, v_dot)
ocp.set_der(w, w_dot)

# Regularised geometric curvature (kappa = w/v algebraically; the sqrt term
# keeps the expression well-defined at v = 0).
k = w / sqrt(v**2 + CURVATURE_EPS**2)

ocp.add_objective(ocp.T)
# Soft boundary-curvature penalty. When curvature_weight = 0 this term
# disappears from the cost (used by non-curvature-aware call sites).
ocp.add_objective(
    curvature_weight * (
        (ocp.at_t0(k) - kappa_start)**2
        + (ocp.at_tf(k) - kappa_end)**2
    )
)
# Soft replacement for the original v(0)=w(0)=0 hard constraint.
ocp.add_objective(
    initial_state_slack_weight * (
        ocp.at_t0(v)**2 + ocp.at_t0(w)**2
    )
)
ocp.subject_to(ocp.at_tf(x)==final_x)
ocp.subject_to(ocp.at_tf(y)==final_y)
ocp.subject_to(ocp.at_tf(theta)==final_angle)

ocp.subject_to( (-max_linear_acc <= v_dot) <= max_linear_acc )
ocp.subject_to( (-max_rot_acc <= w_dot) <= max_rot_acc )
ocp.subject_to( (lower_speed <= v) <= upper_speed )
ocp.subject_to( (-max_rot_speed <= w) <= max_rot_speed )

# Boundary state constraint: relaxed to (x, y, theta) only so that v(0) and
# w(0) are free -- required for a non-zero start curvature to be achievable.
# Original formulation kept commented for reference:
# X = vertcat(x, y, theta, v, w)
# ocp.subject_to(ocp.at_t0(X)==X_0)
ocp.subject_to(ocp.at_t0(x)==X_0[0])
ocp.subject_to(ocp.at_t0(y)==X_0[1])
ocp.subject_to(ocp.at_t0(theta)==X_0[2])

# Pick a solution method
options = {"ipopt": {"print_level": 0}}
options["expand"] = True
options["print_time"] = False
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))

ocp.set_value(X_0, current_X)
ocp.set_value(final_x, end_x)
ocp.set_value(final_y, end_y)
ocp.set_value(final_angle, end_angle)
ocp.set_value(kappa_start, 0.0)
ocp.set_value(kappa_end, 0.0)
ocp.set_value(curvature_weight, 0.0)
ocp.set_value(initial_state_slack_weight, 1.0e6)
# Solve
#sol = ocp.solve()

#tsa, xsol = sol.sample(x, grid='control')

def return_poses(_xf, _yf, _thetaf, _x0, _max_vel, _min_vel, _max_ang_vel, _max_lin_acc, _max_ang_acc,
                 _kappa_start=None, _kappa_end=None, _curvature_weight=0.0):
    ocp.set_value(X_0, _x0)
    ocp.set_value(final_x, _xf)
    ocp.set_value(final_y, _yf)
    ocp.set_value(final_angle, _thetaf)
    ocp.set_value(upper_speed, _max_vel)
    ocp.set_value(lower_speed, _min_vel)
    ocp.set_value(max_rot_speed, _max_ang_vel)
    ocp.set_value(max_linear_acc, _max_lin_acc)
    ocp.set_value(max_rot_acc, _max_ang_acc)

    # Disable the soft penalty when either target is unspecified; in that
    # case also re-tighten the soft v(0)=w(0)=0 constraint so behavior
    # matches the original OCP.
    if _kappa_start is None or _kappa_end is None:
        ocp.set_value(kappa_start, 0.0)
        ocp.set_value(kappa_end, 0.0)
        ocp.set_value(curvature_weight, 0.0)
        ocp.set_value(initial_state_slack_weight, 1.0e6)
    else:
        ocp.set_value(kappa_start, _kappa_start)
        ocp.set_value(kappa_end, _kappa_end)
        ocp.set_value(curvature_weight, _curvature_weight)
        ocp.set_value(initial_state_slack_weight, 0.0)

    sol = ocp.solve()
    tsa, xsol = sol.sample(x, grid='control')
    _, ysol = sol.sample(y, grid='control')
    _, thetasol = sol.sample(theta, grid='control')
    _, vsol = sol.sample(v, grid='control')
    _, wsol = sol.sample(w, grid='control')
    ksol = wsol / np.sqrt(vsol**2 + CURVATURE_EPS**2)

    d = 0
    for i in range(np.size(xsol)-1):
        d+= np.sqrt((xsol[i+1]-xsol[i])**2 + (ysol[i+1]-ysol[i])**2)

    return xsol, ysol, thetasol, ksol, d