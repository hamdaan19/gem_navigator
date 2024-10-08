# MPC for Path Tracking and Obstacle Avoidance
## Note: README under construction
This repository contains code for path tracking of an ackermann-based car using Model Predictive Control (MPC). The MPC controller is designed to take obstacle locations in the surroundings as input and control the vehicle such that it avoids obstacle as well as tracks the reference path. The details on the how the MPC controller was implemented are documented below. The algorithm is simulated in Gazebo and can be directly run from inside a docker container by following the steps mentioed in sections below.

The Gazebo world and model were derived from [Polaris GEM e2 Simulator](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2/-/tree/main). A modified world with obstacles is in [hamdaan19/POLARIS_GEM_e2](https://github.com/hamdaan19/POLARIS_GEM_e2) which is a forked repository. 

<img src="docs/animation.gif" alt="A GIF showcasing obstacle avoidance" style="width:600px;"/>

## Initial Steps
```bash
# Create a catkin workspace
mkdir -p ~/your_ws/src && cd ~/your_ws/src
# Cloning repositories
git clone https://github.com/hamdaan19/gem_navigator.git
# For creating the simulation with obstacles
git clone https://github.com/hamdaan19/POLARIS_GEM_e2.git
```

## Dependencies
1. [Time-Optimal Path Parameterization based on Reachability Analysis (TOPP-RA)](https://github.com/hungpham2511/toppra) | [Paper](https://arxiv.org/abs/1707.07239)
2. [Do-MPC - Robust Optimal Control Toolbox](https://www.do-mpc.com/en/latest/)

## Run with Docker
### Build the Docker Image
```
cd ~/your_ws/src/gem_navigator
# Build the image
xhost +local:docker # For attaching host's display
docker build -t gem_navigator_image . 
# Run the container
./run_container
# Enter into the container with Zsh
docker exec -it gem_nav_container zsh 
# Inside the container
cd /home/gem_ws 
# Initialize and build packages in workspace
catkin init && catkin build
# Source workspace
source /home/gem_ws/devel/setup.zsh
```

### Running the simulation 
The following `roslaunch` command needs to be run inside the container after building the catkin workspace. The file `navigator.launch` creates a world with obstacles, spawns the car, runs the necessary nodes for path tracking, trajectory planning, etc. 
```
roslaunch gem_navigator navigator.launch
```
In a new terminal, run the node which publishes path that needs to be tracked by the MPC controller. This node sends a set of waypoints to the trajectory planner which in turn plans a trajectory and passes it to the controller. 
```
rosrun gem_navigator path_publisher.py
```
After running the above command, the car will begin to move. 

## ROS Nodes Description 
| Sl. no. | Name | Description | Topics Subscribed | Topics Published |
| --- | --- | --- | --- | --- |
| 1. | mpc_controller.py | Contains class for MPC's implementation | /reference_path (`nav_msgs/Path`), closest_obstacle_location (`geometry_msgs/Point`), /gem/base_footprint/odom (`nav_msgs/Odometry`) | /gem/ackermann_cmd (`ackermann_msgs/AckermannDrive`) |
| 2. | time_optimal_trajectory_planner.py | Plans a kino-dynamically feasible trajectory with given set of waypoints | /path_waypoints (`geometry_msgs/PoseArray`) | /reference_path (`nav_msgs/Path`) | 
| 3. | path_publisher.py | Publishes a set of waypoints | - | /path_waypoints (`geometry_msgs/PoseArray`) |
| 4. | esdf_simulator.cc | A simulator for an ESDF. Broadcasts the location of the closest obstacle to the robot's state | /gem/base_footprint/odom (`nav_msgs/Odometry`) | /closest_obstacle_location (`geometry_msgs/Point`) |

## Parameters
* The parameters for the MPC controller are contained in `parameters.yaml` which is uploaded to the ROS Parameter Server in `launch/navigator.launch`. 
* Waypoints are contained `waypoints/wps.csv`. 

## Dynamic Model 
<!-- ![image](docs/car_dynamics.png) -->
<img src="docs/car_dynamics.png" alt="dynamical model" style="width:700px;"/>

## Optimization Problem Definition for MPC
<img src="docs/mpc_objective_function.png" alt="MPC optimization problem" style="width:700px;"/>

### Adding Soft constraints to the Optimization Problem
In the equation below, epsilon is a non-negative slack variable with an upper bound (which determines the maximum violation of the constraint). Epsilon multiplied by a large penalty is added to the objective function. 
<img src="docs/soft_constaints.png" alt="MPC optimization problem" style="width:600px;"/>

Here d is the radius of the circle surrounding the obstacle. The above (soft) constraint ensures that the controller will not allow the euclidean distance between the vehicle and the center of the obstacle to become lesser than d (with the maximum possible violation of epsilon_m). 

## Contributer
* [Mohammad Hamdaan](https://www.linkedin.com/in/mhamdaan/)
