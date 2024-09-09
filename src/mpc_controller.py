#!/usr/bin/env python3
import numpy as np
import sys
import os

import do_mpc
from casadi import *

import rospy
import rospkg
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion

import math

from ackermann_msgs.msg import AckermannDrive

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("gem_navigator"))

from src.callbacks import Callback

cb = Callback()

T_STEP = 0.5
N_HORIZON = 20

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
        print("In tvp_function, t: ", t)
        idx = int(t/self.t_step)
        for k in range(self.n_horizon+1): 
            self.tvp_template['_tvp', k, 'reference_path_pos_x'] = self.path.poses[idx+k].pose.position.x
            self.tvp_template['_tvp', k, 'reference_path_pos_y'] = self.path.poses[idx+k].pose.position.y
            print("k: ", k, "idx: ", idx, "x_ref: ", self.path.poses[k+idx].pose.position.x, "y_ref: ", self.path.poses[k+idx].pose.position.y)

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

        setup_mpc = {
            'n_horizon': self.n_horizon,
            't_step': self.t_step,
            'n_robust': 0,
            'store_full_solution': True,
        }
        self.mpc.set_param(**setup_mpc)

        # Setting the objective function
        lagrange_term = 1.0*(self.model.tvp['reference_path_pos_x']-self.model.x['pos_x'])**2 + 1.0*(self.model.tvp['reference_path_pos_y']-self.model.x['pos_y'])**2 
        meyer_term = 1e-3*(self.model.tvp['reference_path_theta']-(self.model.x['theta']))**2
        self.mpc.set_rterm (
            alpha=1,
            throttle=1e-2
        )
        self.mpc.set_objective(lterm=lagrange_term, mterm=meyer_term)

        self.tvp_template = self.mpc.get_tvp_template()

        self.mpc.set_tvp_fun(self.tvp_function)

        # Setting the constraints for the objective function
        # Velocity limits
        self.mpc.bounds['lower','_x', 'vel'] = 0.0
        self.mpc.bounds['upper','_x', 'vel'] = 5.0
        # Throttle limits
        self.mpc.bounds['lower','_u', 'throttle'] = -2.0
        self.mpc.bounds['upper','_u', 'throttle'] = 2.0
        # Steering angle limits
        self.mpc.bounds['lower','_u', 'alpha'] = -math.pi/16
        self.mpc.bounds['upper','_u', 'alpha'] = math.pi/16
        
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

def callback(data):
    mpc = MPC_Controller(data, n_horizon=N_HORIZON, t_step=T_STEP)
    # pos x, pos y, velocity, theta (yaw)
    x0 = np.array([cb.state_position_x, cb.state_position_y, cb.state_velocity, cb.state_orientation]).reshape(-1,1)
    mpc.mpc.x0 = x0

    mpc.mpc.u0['alpha'] = 0.0 
    mpc.mpc.u0['throttle'] = 0.0

    mpc.mpc.set_initial_guess()

    n_reference_points = len(data.poses)
    
    rate = rospy.Rate(int(10/T_STEP))
    for i in range(n_reference_points - N_HORIZON): 
        x0 = np.array([cb.state_position_x, cb.state_position_y, cb.state_velocity, cb.state_orientation]).reshape(-1,1)
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

    print("exiting...")


if __name__ == "__main__": 

    rospy.init_node("mpc_controller_node", anonymous=False)

    reference_path_sub = rospy.Subscriber("/reference_path", Path, callback, queue_size=10)
    state_sub = rospy.Subscriber("/gem/base_footprint/odom", Odometry, cb.state_callback, queue_size=10)
    ackermann_pub = rospy.Publisher('/gem/ackermann_cmd', AckermannDrive, queue_size=1)

    rospy.spin()