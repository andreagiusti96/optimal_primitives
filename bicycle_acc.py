from rockit import *
from casadi import *

import numpy as np


nx = 5
nu = 2
Nhor = 20

#Position at the rear axle
starting_angle = 0.0
start_x = 0.0
start_y = 0.0
start_v = 0.0
start_delta = 0.0

end_angle = 0.0
end_x = 1.0
end_y = 0.0

current_X = vertcat(start_x,start_y,starting_angle,start_v,start_delta)

# init free time OCP
T0 = 0.0

ocp = Ocp(T=FreeTime(T0))

x = ocp.state()     # x position 
y = ocp.state()     # y position
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
max_rot_speed = ocp.parameter()
max_linear_acc = ocp.parameter()
max_rot_acc = ocp.parameter()
max_steering_angle = ocp.parameter()
L = ocp.parameter()     # wheelbase of the bicycle

ocp.set_der(x, (v*cos(theta)))
ocp.set_der(y, (v*sin(theta)))
ocp.set_der(theta, (v/L)*tan(delta))
ocp.set_der(v, v_dot)
ocp.set_der(delta, delta_dot)

ocp.add_objective(ocp.T)
# Box constraints on the states and controls
ocp.subject_to( (-max_linear_acc <= v_dot) <= max_linear_acc )
ocp.subject_to( (-max_rot_speed <= delta_dot) <= max_rot_speed )
ocp.subject_to( (lower_speed <= v) <= upper_speed )
ocp.subject_to( (-max_steering_angle <= delta) <= max_steering_angle )
# Initial condition
X = vertcat(x, y, theta, v, delta)
ocp.subject_to(ocp.at_t0(X)==X_0)
# Terminal condition
ocp.subject_to(ocp.at_tf(x)==final_x)
ocp.subject_to(ocp.at_tf(y)==final_y)
ocp.subject_to(ocp.at_tf(theta)==final_angle)





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
# Solve
#sol = ocp.solve()

#tsol, xsol = sol.sample(x, grid='control')

def return_poses(_xf, _yf, _thetaf, _x0, _max_vel, _min_vel, _max_ang_vel, _max_lin_acc, _max_ang_acc, _wheelbase, _max_steer_ang):
    ocp.set_value(X_0, _x0)
    ocp.set_value(final_x, _xf)
    ocp.set_value(final_y, _yf)
    ocp.set_value(final_angle, _thetaf)
    ocp.set_value(upper_speed, _max_vel)
    ocp.set_value(lower_speed, _min_vel)
    ocp.set_value(max_rot_speed, _max_ang_vel)
    ocp.set_value(max_linear_acc, _max_lin_acc)
    ocp.set_value(max_rot_acc, _max_ang_acc)
    ocp.set_value(L, _wheelbase)
    ocp.set_value(max_steering_angle, _max_steer_ang)
    sol = ocp.solve()
    tsol, xsol = sol.sample(x, grid='control')
    _, ysol = sol.sample(y, grid='control')
    _, thetasol = sol.sample(theta, grid='control')


    # Poses along the curve
    ds = np.hypot(np.diff(xsol), np.diff(ysol))
    curve_pos = np.cumsum(ds)   # Cumulative distance along the curve
    curve_pos = np.insert(curve_pos, 0, 0.0)  # Add starting position
    total_d = curve_pos[-1]
    return xsol, ysol, thetasol, total_d