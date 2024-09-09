import numpy as np
from scipy.optimize import minimize, Bounds, NonlinearConstraint, BFGS

import rospy
import math

class ModelPredictiveController():
    def __init__(self, max_vel, max_acc, max_turn_angle, intial_state, time_horizon_length=10, delta_t=1e-1): 
        # Time step
        self.delta_t = delta_t
        
        # Setting the time horizon length
        self.time_horizon_length = time_horizon_length
        
        # Initializing the optimization variable
        self.opt_var = np.zeros((6*self.time_horizon_length))

        # initial state
        self.x_init = intial_state

        # Placeholder for bounds for the optimization variable
        # self.bounds = None

        # Placeholder for bounds for the nonlinear constraint due to motion model 
        # self.nonlinear_constraints = None

        # Kino-dynamic bounds
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.max_turn_angle_in_radians = (max_turn_angle/180) * math.pi

        # Wheelbase of the vehicle
        self.wheel_base = 1.5

        # Time step
        self.time_step = 0

    def set_initial_state(self, x_init):
        self.x_init = x_init

    def set_bounds(self): 
        # Initializing empty arrays for upper and lower bounds
        upper_bound = []
        lower_bound = [] 
        # Adding upper and lower bounds
        for i in range(0, self.time_horizon_length): 
            upper_bound.append(np.inf)                              # position x         
            upper_bound.append(np.inf)                              # position y           
            upper_bound.append(math.pi)                             # orientation yaw      
            upper_bound.append(self.max_vel)                        # max velocity         
            upper_bound.append(self.max_turn_angle_in_radians)      # max turn angle   
            upper_bound.append(self.max_acc)                        # max acceleration  

            lower_bound.append(-np.inf)                             # position x                                           
            lower_bound.append(-np.inf)                             # position y                                           
            lower_bound.append(-math.pi)                            # orientation yaw                                          
            lower_bound.append(-self.max_vel)                       # min velocity                                                 
            lower_bound.append(-self.max_turn_angle_in_radians)     # min turn angle                                                               
            lower_bound.append(-self.max_acc)                       # min acceleration                                             

        # Adding equality constraint for initial state
        upper_bound[0] = self.x_init[0]
        upper_bound[1] = self.x_init[1]
        upper_bound[2] = self.x_init[2]
        upper_bound[3] = self.x_init[3]
        lower_bound[0] = self.x_init[0]
        lower_bound[1] = self.x_init[1]
        lower_bound[2] = self.x_init[2]
        lower_bound[3] = self.x_init[3]

        self.bounds = Bounds(lower_bound, upper_bound)

    def motion_model(self, opt_var, k): 
        # k is the time step
        idx = 6*k
        new_state = [
            opt_var[idx+0] + opt_var[idx+3]*math.cos(opt_var[idx+2])*self.delta_t,
            opt_var[idx+1] + opt_var[idx+3]*math.sin(opt_var[idx+2])*self.delta_t,
            opt_var[idx+2] + (opt_var[idx+3]*math.tan(opt_var[idx+4])*self.delta_t)/self.wheel_base,
            opt_var[idx+3] + opt_var[idx+5]*self.delta_t
        ]

        return new_state

    def motion_model_constraints(self, opt_var): 

        residue = []
        for k in range(0, self.time_horizon_length-1):

            idx = 6*k

            residue.append(opt_var[idx+6] - (opt_var[idx+0] + opt_var[idx+3]*math.cos(opt_var[idx+2])*self.delta_t))
            residue.append(opt_var[idx+7] - (opt_var[idx+1] + opt_var[idx+3]*math.sin(opt_var[idx+2])*self.delta_t))
            residue.append(opt_var[idx+8] - (opt_var[idx+2] + (opt_var[idx+3]*math.tan(opt_var[idx+4])*self.delta_t)/self.wheel_base))
            residue.append(opt_var[idx+9] - (opt_var[idx+3] + opt_var[idx+5]*self.delta_t))

        return residue 

    def objective_function(self, opt_var): 
        # We minimize cross track error and orientation error

        total_residue = 0

        for k in range(0, self.time_horizon_length-1): 
            # k is the timestep
            ref_x, ref_y, ref_yaw = self.get_reference_path(k+1)
            
            new_state = self.motion_model(opt_var, k)

            diff = np.array ([
                ref_x   - new_state[0],
                ref_y   - new_state[1],
                ref_yaw - new_state[2]
            ])

            residue = np.linalg.norm(diff, ord=2)**2

            total_residue += residue

        return total_residue

    def setup_optimizer(self):

        self.set_bounds()
        self.nonlinear_constraints = NonlinearConstraint(self.motion_model_constraints, 0.0, 0.0, jac='2-point', hess=BFGS())

    def minimize(self):
        optim_result = minimize(self.objective_function, self.opt_var, method='trust-constr',  jac="2-point", hess=BFGS(),
                constraints=[self.nonlinear_constraints],
               options={'verbose': 1}, bounds=self.bounds)

        return optim_result

    def get_reference_path(self, time_step): 
        path = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [7.6, 0.0, 0.0]
        ])

        return path[time_step][0], path[time_step][1], path[time_step][2] 

    def main(self):
        # self.set_intial_state([0.0, 0.0, 0.0, 0.0])
        self.setup_optimizer()
        result = self.minimize()

        print(result.x)     

        for i in range(self.time_horizon_length):
            idx = 6*i
            print("x: ", result.x[idx], " y: ", result.x[idx+1], " acc: ", result.x[idx+5], "vel: ", result.x[idx+3])

        # print(self.motion_model_constraints(result.x))