from rockit import *
from casadi import *

import numpy as np


nx = 5
nu = 2
Nhor = 20

# Regularisation used inside the geometric-curvature expression k = w / sqrt(v^2 + eps^2)
# to keep it finite (and differentiable) when v ~ 0.
CURVATURE_EPS = 1e-3


# init free time OCP
T0 = 0.0
ocp = Ocp(T=FreeTime(T0))

x = ocp.state()     # x position rear axle
y = ocp.state()     # y position rear axle
theta = ocp.state() # heading angle
v = ocp.state()     # linear velocity
delta = ocp.state() # steering angle

v_dot = ocp.control()
delta_dot = ocp.control()

X_0 = ocp.parameter(nx)
final_x = ocp.parameter()
final_y = ocp.parameter()
final_angle = ocp.parameter()
upper_speed = ocp.parameter()
lower_speed = ocp.parameter()
max_steer_speed = ocp.parameter()
max_linear_acc = ocp.parameter()
max_steering_angle = ocp.parameter()
kappa_start = ocp.parameter()
kappa_end = ocp.parameter()
curvature_weight = ocp.parameter()
L = ocp.parameter()     # wheelbase of the bicycle
# Soft weight for v(0) = w(0) = 0. Set high (e.g. 1e6) to reproduce the
# original hard-fixed initial state; set to 0 when the curvature soft
# constraint is active so the OCP can pick a non-zero start velocity.
initial_state_slack_weight = ocp.parameter()

ocp.set_der(x, (v*cos(theta)))
ocp.set_der(y, (v*sin(theta)))
ocp.set_der(theta, (v/L)*tan(delta))
ocp.set_der(v, v_dot)
ocp.set_der(delta, delta_dot)

# Regularised geometric curvature (kappa = w/v algebraically; the sqrt term
# keeps the expression well-defined at v = 0).
k = (v/L)*tan(delta) / sqrt(v**2 + CURVATURE_EPS**2)

# Box constraints on the states and controls
ocp.subject_to( (-max_linear_acc <= v_dot) <= max_linear_acc )
# ocp.subject_to( (-1e-9 <= v_dot) <= 1e-9 )  # const velocity
ocp.subject_to( (-max_steer_speed <= delta_dot) <= max_steer_speed )
ocp.subject_to( (lower_speed <= v) <= upper_speed )
ocp.subject_to( (-max_steering_angle <= delta) <= max_steering_angle )


# Initial condition
ocp.subject_to(ocp.at_t0(x)==X_0[0])
ocp.subject_to(ocp.at_t0(y)==X_0[1])
ocp.subject_to(ocp.at_t0(theta)==X_0[2])
# ocp.subject_to(ocp.at_t0(v)==X_0[3])
# ocp.subject_to(ocp.at_t0(delta)==X_0[4])
ocp.subject_to(ocp.at_t0(k)==kappa_start)


# Terminal condition
ocp.subject_to(ocp.at_tf(x)==final_x)
ocp.subject_to(ocp.at_tf(y)==final_y)
ocp.subject_to(ocp.at_tf(theta)==final_angle)
# ocp.subject_to(ocp.at_tf(delta)==X_0[4])
ocp.subject_to(ocp.at_tf(k)==kappa_end)

ocp.add_objective(ocp.T)
ocp.add_objective(ocp.integral(delta_dot**2)*1.0)


# Pick a solution method
options = {"ipopt": {"print_level": 0}}
options["expand"] = True
options["print_time"] = False
ocp.solver('ipopt',options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=Nhor,M=1,intg='rk'))



def return_poses(_xf, _yf, _thetaf, _x0, _max_vel, _min_vel, _max_ang_vel, _max_lin_acc, _max_ang_acc, _wheelbase, _max_steer_ang, _max_steer_speed,
                 _kappa_start=None, _kappa_end=None, _curvature_weight=0.0):
    
    ocp.set_value(X_0, _x0)
    ocp.set_value(final_x, _xf)
    ocp.set_value(final_y, _yf)
    ocp.set_value(final_angle, _thetaf)
    ocp.set_value(upper_speed, _max_vel)
    ocp.set_value(lower_speed, _min_vel)
    ocp.set_value(max_steer_speed, _max_steer_speed)
    ocp.set_value(max_linear_acc, _max_lin_acc)
    ocp.set_value(L, _wheelbase)
    ocp.set_value(max_steering_angle, _max_steer_ang)
    
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
        
    # Solve the optimal control problem and extract the solution
    sol = ocp.solve()
    tsol, xsol = sol.sample(x, grid='control')
    _, ysol = sol.sample(y, grid='control')
    _, thetasol = sol.sample(theta, grid='control')
    _, vsol = sol.sample(v, grid='control')
    _, delta_sol = sol.sample(delta, grid='control')
    _, ksol = sol.sample(k, grid='control')


    # Poses along the curve
    ds = np.hypot(np.diff(xsol), np.diff(ysol))
    curve_pos = np.cumsum(ds)   # Cumulative distance along the curve
    curve_pos = np.insert(curve_pos, 0, 0.0)  # Add starting position
    total_d = curve_pos[-1]
    
    return xsol, ysol, thetasol, ksol, total_d