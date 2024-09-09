import rospy
from tf.transformations import euler_from_quaternion
import math

class Callback():
    def __init__(self):
        self.state_position_x = 0.0
        self.state_position_y = 0.0
        self.state_orientation = 0.0
        self.state_velocity = 0.0

    def state_callback(self, data):
        self.state_position_x = data.pose.pose.position.x
        self.state_position_y = data.pose.pose.position.y

        quat = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

        r, p, yaw = euler_from_quaternion(quat)

        self.state_orientation = yaw

        vel_x = data.twist.twist.linear.x
        vel_y = data.twist.twist.linear.y

        self.state_velocity = math.sqrt(vel_x**2 + vel_y**2)
