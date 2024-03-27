import rclpy
from geometry_msgs.msg import Twist
from approxeng.input.selectbinder import ControllerResource
from time import sleep

def twist_publisher():
    rclpy.init()
    node = rclpy.create_node('twist_publisher')

    publisher = node.create_publisher(Twist, 'cmd_vel', 10)

    msg = Twist()


    while rclpy.ok():
        while True:

            with ControllerResource() as joystick:
                left_x = float(joystick['lx'])
                #print(left_x, type(left_x))
                #left_x = 0.5
                left_y = float(joystick['ly'])
                #left_y = 0.5

                msg.linear.x = left_x
                msg.linear.y = left_y
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0
                
                publisher.publish(msg)
                node.get_logger().info('sending')
                #ode.get_logger().info('Publishing Twist: linear.x={}, angular.z={}'.format(msg.linear.x, msg.angular.z))
                #rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    twist_publisher()
