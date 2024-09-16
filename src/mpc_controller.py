#!/usr/bin/env python3
import numpy as np
import sys
import os

import do_mpc
from casadi import *

import rospy
import rospkg
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

import matplotlib.pyplot as plt

import math

from ackermann_msgs.msg import AckermannDrive

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("gem_navigator"))

from src.callbacks import Callback

cb = Callback()

T_STEP = rospy.get_param("/gem_navigator/t_step")
N_HORIZON = rospy.get_param("/gem_navigator/n_horizon")
MAX_VEL = rospy.get_param("/gem_navigator/max_velocity")

class MPC_Controller(): 
    def __init__(self, path, n_horizon=20, t_step=0.1):

        # Trajectory which needs to be tracked
        self.path = path

        # Length of time horizon and time step
        self.n_horizon = n_horizon
        self.t_step = t_step

        self.model = do_mpc.model.Model('continuous')
        # Placeholder for MPC controller
        self.mpc = None

        self.wheel_base = 1.75
        self.acceleration = 2.0

        self.configure_controller()

    def tvp_function(self, t): 

        idx = int(t/self.t_step)
        for k in range(self.n_horizon+1): 
            self.tvp_template['_tvp', k, 'reference_path_pos_x'] = self.path.poses[idx+k].pose.position.x
            self.tvp_template['_tvp', k, 'reference_path_pos_y'] = self.path.poses[idx+k].pose.position.y
            quat = (
                self.path.poses[k+idx].pose.orientation.x,
                self.path.poses[k+idx].pose.orientation.y,
                self.path.poses[k+idx].pose.orientation.z,
                self.path.poses[k+idx].pose.orientation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quat)

            # unwrapping yaw
            # if (yaw < 0):
            #     yaw += 2*math.pi

            self.tvp_template['_tvp', k, 'reference_path_theta'] = yaw

            # Setting obstacle information 
            self.tvp_template['_tvp', k, 'obstacle_1_pos', 0] = cb.obs_x
            self.tvp_template['_tvp', k, 'obstacle_1_pos', 1] = cb.obs_y

        return self.tvp_template


    def configure_controller(self): 
        ########################################
        #### CREATING THE MODEL ####
        ########################################
        # Defining state variables
        pos_x = self.model.set_variable(var_type='_x', var_name='pos_x', shape=(1,1))   # Vehicle's position Y
        pos_y = self.model.set_variable(var_type='_x', var_name='pos_y', shape=(1,1))   # Vehicle's position X
        vel = self.model.set_variable(var_type='_x', var_name='vel', shape=(1,1))       # Velocity in X direction in vehicle body frame
        theta = self.model.set_variable(var_type='_x', var_name='theta', shape=(1,1))   # Yaw angle

        # Defining control variables
        alpha = self.model.set_variable(var_type='_u', var_name='alpha')         # Steering angle
        throttle = self.model.set_variable(var_type='_u', var_name='throttle')   # Throttle [-1, 1]

        # Defining time varying parameters
        reference_path_pos_x = self.model.set_variable(var_type='_tvp', var_name='reference_path_pos_x')
        reference_path_pos_y = self.model.set_variable(var_type='_tvp', var_name='reference_path_pos_y')
        reference_path_theta = self.model.set_variable(var_type='_tvp', var_name='reference_path_theta')
        obstacle_pos_1_x = self.model.set_variable(var_type="_tvp", var_name='obstacle_1_pos', shape=(2,1))
        # obstacle_pos_y = self.model.set_variable(var_type="_tvp", var_name='obstacle_pos_y', shape=(1,1))

        # Setting the RHS for the differential equation x_dot = f(x,u)
        self.model.set_rhs('pos_x', vel*cos(theta))
        self.model.set_rhs('pos_y', vel*sin(theta))
        self.model.set_rhs('theta', vel*tan(alpha)/self.wheel_base)
        self.model.set_rhs('vel', throttle)

        self.model.setup()

        print(self.model.x.labels())

        #########################################
        #### CONFIGURING THE MPC CONTROLLER #####
        #########################################
        self.mpc = do_mpc.controller.MPC(self.model)

        # Solver Settings
        self.mpc.settings.n_horizon = self.n_horizon
        self.mpc.settings.t_step = self.t_step
        self.mpc.settings.n_robust = 0
        self.mpc.settings.store_full_solution = True

        # Suppress IPOPT output
        self.mpc.settings.supress_ipopt_output()

        # Setting the objective function
        lagrange_term = 1.0*(self.model.tvp['reference_path_pos_x']-self.model.x['pos_x'])**2 + 1.0*(self.model.tvp['reference_path_pos_y']-self.model.x['pos_y'])**2 + 1e-6*(self.model.tvp['reference_path_theta']-(self.model.x['theta']))**2 
        meyer_term = 1.0*(self.model.tvp['reference_path_pos_x']-self.model.x['pos_x'])**2 + 1.0*(self.model.tvp['reference_path_pos_y']-self.model.x['pos_y'])**2 + 1e-3*(self.model.tvp['reference_path_theta']-(self.model.x['theta']))**2
        self.mpc.set_rterm (
            alpha=1,
            throttle=1e-2
        )
        self.mpc.set_objective(lterm=lagrange_term, mterm=meyer_term)

        # Set scaling for state variables
        # self.mpc.scaling['_x', 'pos_x'] = 1.0
        # self.mpc.scaling['_x', 'pos_y'] = 1.0
        # self.mpc.scaling['_x', 'vel'] = 1.0
        # self.mpc.scaling['_x', 'theta'] = 1e-3

        self.tvp_template = self.mpc.get_tvp_template()

        self.mpc.set_tvp_fun(self.tvp_function)

        # Setting the constraints for the objective function
        # Velocity limits
        self.mpc.bounds['lower','_x', 'vel'] = 0.0
        self.mpc.bounds['upper','_x', 'vel'] = MAX_VEL
        # Throttle limits
        self.mpc.bounds['lower','_u', 'throttle'] = -2.0
        self.mpc.bounds['upper','_u', 'throttle'] = 2.0
        # Steering angle limits
        self.mpc.bounds['lower','_u', 'alpha'] = -math.pi/16
        self.mpc.bounds['upper','_u', 'alpha'] = math.pi/16

        # Setting Constraints for obstacle avoidance
        obstacle_constraint_expr = -1*(self.model.x['pos_x'] - self.model.tvp['obstacle_1_pos', 0])**2 - (self.model.x['pos_y'] - self.model.tvp['obstacle_1_pos', 1])**2

        self.mpc.set_nl_cons('T_R_UB', obstacle_constraint_expr, ub=-4.0, soft_constraint=True, penalty_term_cons=1e9, maximum_violation=0.1)
        
        self.mpc.setup()

def simulator(u, x, t_step=0.1):
    print(type(u), type(x))
    delta_x = np.array([
        x[2]*cos(x[3]),
        x[2]*sin(x[3]),
        2*u[1],
        x[2]*tan(u[0])/1.5
    ]).reshape(-1,1)
    x_next = x + delta_x*t_step
    return x_next

def get_points_from_path(path):
    points = np.array([[path.poses[0].pose.position.x, path.poses[0].pose.position.y]])
    for i in range(1, len(path.poses)):
        pt = np.array([[path.poses[i].pose.position.x, path.poses[i].pose.position.y]])
        points = np.vstack((points, pt))

    return points

def get_crosstrack_error(points, x, y):
    q_pt = np.array([x,y]).reshape(-1,1)
    distances = []
    for idx in range(0, points.shape[0]-1):
        pt = points[idx].reshape(-1,1)        

        dist = np.linalg.norm(q_pt-pt)
        distances.append(dist)

    min_idx = np.argmin(distances)
    pt1 = points[min_idx]
    pt2 = points[min_idx+1]

    x1 = pt1[0]
    y1 = pt1[1]
    x2 = pt2[0]
    y2 = pt2[1]

    
    m = (y2-y1)/(x2-x1)
    c = y1 - m*x1
        
    min_dist = min(distances)

    if (y >= m*x+c):
        return min_dist
    else:
        return -1*min_dist



def callback(data):
    path_points = get_points_from_path(data)

    mpc = MPC_Controller(data, n_horizon=N_HORIZON, t_step=T_STEP)
    # pos x, pos y, velocity, theta (yaw)
    x0 = np.array([cb.state_position_x, cb.state_position_y, cb.state_velocity, cb.state_orientation]).reshape(-1,1)
    mpc.mpc.x0 = x0

    mpc.mpc.u0['alpha'] = 0.0 
    mpc.mpc.u0['throttle'] = 0.0

    mpc.mpc.set_initial_guess()

    n_reference_points = len(data.poses)

    cte_array = []
    time_step_arr = []
    
    rate = rospy.Rate(int(10/T_STEP))
    for i in range(n_reference_points - N_HORIZON): 
        u0 = mpc.mpc.make_step(x0)
        # x0 = simulator(u0, x0)

        j = 0
        v_new = float(x0[2] + u0[1]*T_STEP)
        while ((not rospy.is_shutdown()) and j < 10): 
            j += 1

            # Defining ackermann_msg
            ackermann_msg = AckermannDrive()
            ackermann_msg.steering_angle_velocity = 0.0
            ackermann_msg.acceleration            = float(u0[1])
            ackermann_msg.jerk                    = 0.0
            ackermann_msg.speed                   = v_new 
            ackermann_msg.steering_angle          = float(u0[0])

            # publishing
            ackermann_pub.publish(ackermann_msg)

            # Letting time elapse
            rate.sleep()

        # Calculating Root Mean Squared Deviation
        x0 = np.array([cb.state_position_x, cb.state_position_y, cb.state_velocity, cb.state_orientation]).reshape(-1,1)
        # dist = math.sqrt( (data.poses[i].pose.position.x - cb.state_position_x)**2 + (data.poses[i].pose.position.y - cb.state_position_y)**2 )
        cte = get_crosstrack_error(path_points, cb.state_position_x, cb.state_position_y)
        print("Crosstrack Error: ", get_crosstrack_error(path_points, cb.state_position_x, cb.state_position_y))
        cte_array.append(cte)
        time_step_arr.append(i)

    cte_arr_np = np.array(cte_array)
    time_step_arr_np = np.array(time_step_arr)

    plt.plot(time_step_arr_np, cte_arr_np)
    plt.xlabel("Time Step")
    plt.ylabel("Crosstrack Error")
    plt.show()
    
    print("exiting...")


if __name__ == "__main__": 

    rospy.init_node("mpc_controller_node", anonymous=False)

    reference_path_sub = rospy.Subscriber("/reference_path", Path, callback, queue_size=10)
    state_sub = rospy.Subscriber("/gem/base_footprint/odom", Odometry, cb.state_callback, queue_size=10)
    obstacle_sub = rospy.Subscriber("/closest_obstacle_location", Point, cb.obstacle_callback, queue_size=10)
    ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)

    rospy.spin()