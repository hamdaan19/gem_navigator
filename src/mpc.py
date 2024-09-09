import numpy as np
from scipy.optimize import minimize, Bounds, NonlinearConstraint, BFGS

import rospy
import math

delta_t = 1.0
wheel_base = 1.5 # meters

def f1(opt_var):
    residue = opt_var[6] - ( opt_var[0] + opt_var[3]*math.cos(opt_var[2])*delta_t )
    return residue 

def f2(opt_var):
    residue = opt_var[7] - ( opt_var[1] + opt_var[3]*math.sin(opt_var[2])*delta_t )
    return residue 

def f3(opt_var):
    residue = opt_var[8] - ( opt_var[2] + (opt_var[3]*math.tan(opt_var[4])*delta_t)/wheel_base )
    return residue 

def f4(opt_var):
    residue = opt_var[9] - ( opt_var[3] + opt_var[5]*delta_t )
    return residue 

def obj_f1(opt_var): 
    residue = opt_var[0] + opt_var[3]*math.cos(opt_var[2])*delta_t
    return residue

def obj_f2(opt_var): 
    residue = opt_var[1] + opt_var[3]*math.sin(opt_var[2])*delta_t
    return residue

def obj_f3(opt_var): 
    residue = opt_var[2] + (opt_var[3]*math.tan(opt_var[4])*delta_t)/wheel_base
    return residue

def motion_model_function(opt_var):
    return [f1(opt_var), f2(opt_var), f3(opt_var), f4(opt_var)]

def objective_function(opt_var): 
    x_1 = 0.0
    y_1 = 0.0
    theta_1 = 0.0
    x_2 = 0.1
    y_2 = 0.1
    theta_2 = 0.015

    difference_1 = np.array([
        [x_2 - obj_f1(opt_var)],
        [y_2 - obj_f2(opt_var)],
        [theta_2 - obj_f3(opt_var)]
    ])

    residue = float(np.linalg.norm(difference_1, ord=2)**2)

    return residue

def optimize(): 
    # Let us say the length of time horizon is 2

    # Assuming our initial state starts at 0
    x_init = np.array([0.0, 0.0, 0.0, 0.0]) # position x-axis, position y-axis, theta, velocity

    opt_var = np.zeros((12,))
    opt_var[:4] = x_init

    # Setting the bounds
    bounds = Bounds([0.0, 0.0, 0.0, 0.1, -math.pi/8, -2.0, -np.inf, -np.inf, -math.pi, -10.0, -math.pi/8, -2.0], 
                    [0.0, 0.0, 0.0, 0.1,  math.pi/8,  2.0,  np.inf,  np.inf,  math.pi,  10.0,  math.pi/8,  2.0])

    motion_model_constraint_1 = NonlinearConstraint(motion_model_function, 0.0, 0.0, jac='2-point', hess=BFGS())

    res = minimize(objective_function, opt_var, method='trust-constr',  jac="2-point", hess=BFGS(),
                constraints=[motion_model_constraint_1],
               options={'verbose': 1}, bounds=bounds)

    print(res.x)
    

if __name__ == "__main__": 
    rospy.init_node("path_controller_node", anonymous=False)

    # state_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, cb.pose_callback, queue_size=10)

    optimize()