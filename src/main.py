import rospy
import rospkg

import sys

rp = rospkg.RosPack()
sys.path.insert(1, rp.get_path("gem_navigator"))

from src.model_predictive_controller import ModelPredictiveController

if __name__ == "__main__": 
    rospy.init_node("path_controller_node", anonymous=False)

    mpc = ModelPredictiveController(10.0, 1.0, 20.0, [0.0,0.0,0.0,0.0], 4, 1.0)
    mpc.main()
