import rclpy
# import the Node module from ROS2 Python library
from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('Byakugan')
        # create a timer sending two parameters:
        # - the duration between two callbacks (0.2 seconds)
        # - the timer function (timer_callback)
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        # print a ROS2 log on the terminal with a great message!
        self.get_logger().info("Moe yo Byakugan! Kore ga watashi no nindō yo")


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("Moe yo Byakugan! Kore ga watashi no nindō yo ")
    # Translated to English should be "Blaze Away, Byakugan! This is My Ninja Way!"

    # declare the node constructor
    node = MyNode()
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()  # call the main function
