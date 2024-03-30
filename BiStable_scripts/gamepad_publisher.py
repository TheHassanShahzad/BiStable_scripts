import rclpy
from rclpy.node import Node
from bistable_interfaces.msg import GamepadData
from approxeng.input.selectbinder import ControllerResource
from time import sleep



# ROS 2 Node for publishing target positions
class GamepadPublisher(Node):
    def __init__(self):
        super().__init__('target_positions_publisher')
        self.publisher_ = self.create_publisher(GamepadData, '/gamepad_data', 10)

    def publish_data(self, inc, yv, eq_c, kp_c, ki_c, kd_c, toggle):
        msg = GamepadData()
        msg.inclination = inc
        msg.yaw_vel = yv
        msg.eq_change = eq_c
        msg.kp_change = kp_c
        msg.ki_change = ki_c
        msg.kd_change = kd_c
        msg.tracking_toggle = toggle

        self.publisher_.publish(msg)

def linear_map(input, gain):
    return input * gain

def main(args=None):
    rclpy.init(args=args)
    node = GamepadPublisher()
    tracking_state = False
    # prev_square_state = None
    # prev_cross_state = None
    # prev_circle_state = None
    # prev_triangle_state = None
    prev_L1_state = None
    prev_R1_state = None  
    prev_R2_state = None
    prev_L2_state = None
    inclination = 0.0     # Inclination angle in degrees
    yaw_vel = 0.0         # Yaw velocity in m/s
    max_inclination = 8.0 # Maximum inclination angle in degrees
    max_yaw_vel = 0.05    # Maximum yaw velocity in m/s

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

                    # Map joystick values to inclination angle and yaw velocity
                    inclination = linear_map(left_y, max_inclination)
                    yaw_vel = linear_map(left_x, max_yaw_vel)
                    
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

                    # Press L1 and R1 to toggle tracking
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
                                
                    # L1 to change value by -0.1
                    if L1_state and not prev_L1_state:
                        if square_state:
                            kp_change = -0.1
                        if cross_state:
                            ki_change = -0.1
                        if circle_state:
                            kd_change = -0.1
                        if triangle_state:
                            eq_change = -0.1

                    # L2 to change value by -0.01
                    if L2_state and not prev_L2_state:
                        if square_state:
                            kp_change = -0.01
                        if cross_state:
                            ki_change = -0.01
                        if circle_state:
                            kd_change = -0.01
                        if triangle_state:
                            eq_change = -0.01    

                    # R1 to change value by +0.1
                    if R1_state and not prev_R1_state:
                        if square_state:
                            kp_change = 0.1
                        if cross_state:
                            ki_change = 0.1
                        if circle_state:
                            kd_change = 0.1
                        if triangle_state:
                            eq_change = 0.1 

                    # R2 to change value by +0.01
                    if R2_state and not prev_R2_state:
                        if square_state:
                            kp_change = 0.01
                        if cross_state:
                            ki_change = 0.01
                        if circle_state:
                            kd_change = 0.01
                        if triangle_state:
                            eq_change = 0.01

                    # Update previous button states

                    prev_L1_state = L1_state
                    prev_R1_state = R1_state
                    prev_L2_state = L2_state
                    prev_R2_state = R2_state

                    node.publish_data(inclination, yaw_vel, eq_change, kp_change, ki_change, kd_change, tracking_state)
                    # data_to_be_published = [inclination, yaw_vel, eq_change, kp_change, ki_change, kd_change, tracking_state]
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
