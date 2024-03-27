import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from approxeng.input.selectbinder import ControllerResource
from time import sleep



# ROS 2 Node for publishing target positions
class GamepadPublisher(Node):
    def __init__(self):
        super().__init__('target_positions_publisher')
        self.TwistPublisher_ = self.create_publisher(Twist, '/gamepad_data', 10)
        self.BoolPublisher_ = self.create_publisher(Bool, '/toggle', 10)

    def publish_general(self, lx, ly, lz, ax, ay, az):
        msg = Twist()
        msg.linear.x = lx
        msg.linear.y = ly
        msg.linear.z = lz
        msg.angular.x = ax
        msg.angular.y = ay
        msg.angular.z = az
        self.TwistPublisher_.publish(msg)

    def publish_toggle(self, a):
        msg = Bool()
        msg.data = a
        self.BoolPublisher_.publish(msg)


def main(args=None):
    # global start_time  # Declare start_time as global
    # start_time = time.time()  # Initialize start_time
    rclpy.init(args=args)
    node = GamepadPublisher()
    tracking_state = False
    prev_square_state = None
    prev_cross_state = None
    prev_circle_state = None
    prev_triangle_state = None
    prev_L1_state = None
    prev_R1_state = None  

    while True:

        try:
            with ControllerResource() as joystick:
                while joystick.connected:
                    eq_change = 0.0
                    kp_change = 0.0
                    ki_change = 0.0
                    kd_change = 0.0

                    # Read left joystick axes
                    left_x = float(joystick['lx'])
                    left_y = float(joystick['ly'])
                    #print(left_x, left_y)
                    
                    # Read button states
                    square_state = joystick['square']
                    cross_state = joystick['cross']
                    circle_state = joystick['circle']
                    triangle_state = joystick['triangle']

                    L1_state = joystick['l1']
                    L2_state = joystick['l2']
                    R1_state = joystick['r1']
                    R2_state = joystick['r2']
                    #R3_state = joystick['r3']

                    if L1_state != prev_L1_state:
                        if R1_state:
                            if tracking_state == False:
                                tracking_state = True
                            else:
                                tracking_state = False

                    if R1_state != prev_R1_state:
                        if L1_state:
                            if tracking_state == False:
                                tracking_state = True
                            else:
                                tracking_state = False


                    #print(tracking_state)
                    # Check if button is pressed down
                    if square_state and not prev_square_state:
                        if L1_state:
                            #print("kp -0.1")
                            kp_change = -0.1
                        
                    if cross_state and not prev_cross_state:
                        if L1_state:
                            #print("ki -0.1")
                            ki_change = -0.1

                    if circle_state and not prev_circle_state:
                        if L1_state:
                            #print("kd -0.1")
                            kd_change = -0.1

                    if triangle_state and not prev_triangle_state:
                        if L1_state:
                            #print("eq -0.1")
                            eq_change = -0.1

                    if square_state and not prev_square_state:
                        if L2_state:
                            #print("kp -0.01")
                            kp_change = -0.01
                        
                    if cross_state and not prev_cross_state:
                        if L2_state:
                            #print("ki -0.01")
                            ki_change = -0.01

                    if circle_state and not prev_circle_state:
                        if L2_state:
                            #print("kd -0.01")
                            kd_change = -0.01

                    if triangle_state and not prev_triangle_state:
                        if L2_state:
                            #print("eq -0.01")
                            eq_change = -0.01

                    if square_state and not prev_square_state:
                        if R1_state:
                            #print("kp +0.01")
                            kp_change = 0.1
                        
                    if cross_state and not prev_cross_state:
                        if R1_state:
                            #print("ki +0.01")
                            ki_change = 0.1

                    if circle_state and not prev_circle_state:
                        if R1_state:
                            #print("kd +0.01")
                            kd_change = 0.1

                    if triangle_state and not prev_triangle_state:
                        if R1_state:
                            #print("eq +0.01")
                            eq_change = 0.1

                    if square_state and not prev_square_state:
                        if R2_state:
                            #print("kp +0.1")
                            kp_change = 0.01
                        
                    if cross_state and not prev_cross_state:
                        if R2_state:
                            #print("ki +0.1")
                            ki_change = 0.01

                    if circle_state and not prev_circle_state:
                        if R2_state:
                            #print("kd +0.1")
                            kd_change = 0.01

                    if triangle_state and not prev_triangle_state:
                        if R2_state:
                            #print("eq +0.1")
                            eq_change = 0.01

                    # Update previous button states
                    prev_square_state = square_state
                    prev_cross_state = cross_state
                    prev_circle_state = circle_state
                    prev_triangle_state = triangle_state
                    prev_L1_state = L1_state
                    prev_R1_state = R1_state


                    node.publish_general(left_x, left_y, eq_change, kp_change, ki_change, kd_change)
                    node.publish_toggle(tracking_state)
                    # data_to_be_published = [left_x, left_y, eq_change, kp_change, ki_change, kd_change, tracking_state]
                    # print(data_to_be_published)
                    sleep(0.05)

            print('Connection to joystick lost')
        except IOError:
            # No joystick found, wait for a bit before trying again
            print('Unable to find any joysticks')
            sleep(1.0)

    # Release the video capture and close all windows
    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
