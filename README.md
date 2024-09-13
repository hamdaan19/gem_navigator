# Model Predictive Control for Path Tracking and Obstacle Avoidance of a Car

## Initial Steps
```bash
# Create a catkin workspace
mkdir -p ~/your_ws/src && cd ~/your_ws/src
# Cloning repositories
git clone https://github.com/hamdaan19/gem_navigator.git
# For creating the simulation with obstacles
git clone https://github.com/hamdaan19/POLARIS_GEM_e2.git
```

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
The following `roslaunch` command needs to be run inside the container after building the catkin workspace. The file `navigator.launch` creates a world with static obstacles, spawns the car, runs the necessary nodes for path tracking, trajectory planning, etc. 
```
roslaunch gem_navigator navigator.launch
```
In a new terminal, run the node which publishes path that needs to be tracked by the MPC controller. This node sends a set of waypoints to the trajectory planner which in turn plans a trajectory and passes it to the controller. 
```
rosrun gem_navigator path_publisher.py
```
After running the above command, the car will begin to move. 

## ROS nodes and Topics
