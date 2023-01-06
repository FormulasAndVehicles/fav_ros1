import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

NOISE_BUFFER_SIZE = 50


class Node():
    def __init__(self) -> None:
        rospy.init_node('virtual_vision')
        self.stddev_x = 0.0
        self.stddev_y = 0.0
        self.stddev_z = 0.0
        self.counter = 0
        self.update_noise()
        self.vision_pose_pub = rospy.Publisher('visual_localization/pose',
                                               PoseWithCovarianceStamped,
                                               queue_size=10)
        self.ground_truth_sub = rospy.Subscriber('ground_truth/state', Odometry,
                                                 self.on_odometry)

    def update_noise(self):
        self.stddev_x = rospy.get_param('~stddev_x', self.stddev_x)
        self.stddev_y = rospy.get_param('~stddev_x', self.stddev_y)
        self.stddev_z = rospy.get_param('~stddev_x', self.stddev_z)
        self.noise_x = np.random.normal(0.0, self.stddev_x, NOISE_BUFFER_SIZE)
        self.noise_y = np.random.normal(0.0, self.stddev_y, NOISE_BUFFER_SIZE)
        self.noise_z = np.random.normal(0.0, self.stddev_z, NOISE_BUFFER_SIZE)

    def on_odometry(self, msg: Odometry):
        self.counter += 1
        if self.counter >= NOISE_BUFFER_SIZE:
            self.update_noise()
            self.counter = 0
        msg_out = PoseWithCovarianceStamped()
        msg_out.header = msg.header
        p = msg.pose.pose.position
        # add noise to the ground truth position
        msg_out.pose.pose.position.x = p.x + self.noise_x[self.counter]
        msg_out.pose.pose.position.y = p.y + self.noise_y[self.counter]
        msg_out.pose.pose.position.z = p.z + self.noise_z[self.counter]
        # just copy the orientation data. no noise added
        msg_out.pose.pose.orientation = msg.pose.pose.orientation
        # set the covariance according to specified noise.
        # in the lab this value will be updated dynamically by the EKF
        # prediction/update.
        msg_out.pose.covariance = np.diag([
            self.stddev_x**2, self.stddev_y**2, self.stddev_z**2, 0.0, 0.0, 0.0
        ]).flatten().tolist()
        self.vision_pose_pub.publish(msg_out)

    def run(self):
        rospy.spin()


def main():
    n = Node()
    n.run()


if __name__ == '__main__':
    main()
