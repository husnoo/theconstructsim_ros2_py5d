import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile


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
        self.get_logger().info('I receive: "%s"' % str(msg))


def main(args=None):
    rclpy.init(args=args)
    topic_quiz = TopicQuiz()
    rclpy.spin(topic_quiz)
    topic_quiz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
