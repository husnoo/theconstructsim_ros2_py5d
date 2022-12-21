import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

import numpy as np


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Below should be replaced when porting for ROS2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TopicQuiz(Node):

    def __init__(self):
        super().__init__('topic_quiz_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def listener_callback(self, msg):
        #self.get_logger().info('I receive: "%s"' % str(msg))
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        quaternion = [orient.x, orient.y, orient.z, orient.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.get_logger().info(
            'pos: {} roll: {:.2f} pitch: {:.2f} yaw: {:.2f}'.format(pos, roll, pitch, yaw))


def main(args=None):
    rclpy.init(args=args)
    topic_quiz = TopicQuiz()
    rclpy.spin(topic_quiz)
    topic_quiz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
