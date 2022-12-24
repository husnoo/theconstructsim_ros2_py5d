# import the empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('service_stop')
        # create the Service server object
        # defines the type, name, and callback function
        self.srv = self.create_service(Empty, 'stop', self.empty_callback)
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def empty_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response

        # create a Twist message
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel topic parameter to 0
        msg.linear.x = 0.0
        # define the angular z-axis velocity of /cmd_vel topic parameter to 0
        msg.angular.z = 0.0
        # Publish the message to the topic
        self.publisher_.publish(msg)
        # print a pretty message
        self.get_logger().info('Stop there, Robot!')
        
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
