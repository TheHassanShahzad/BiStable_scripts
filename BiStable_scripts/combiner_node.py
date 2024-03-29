#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bistable_interfaces.msg import GamepadData
from bistable_interfaces.msg import TrackingData
from geometry_msgs.msg import Twist

class Combiner(Node): 
    def __init__(self):
        super().__init__("combiner") 
        self.subscriber1_ = self.create_subscription(GamepadData, "/gamepad_data", self.callback_gamepad_commands, 10)
        self.subscriber2_ = self.create_subscription(TrackingData, "/tracking_data", self.callback_toggle_tracking, 10)
        self.publisher_ = self.create_publisher(Twist, "/combined_data", 10)
        self.timer_ = self.create_timer(0.05, self.final_commands_publisher)
        self.get_logger().info("Combiner has been started")

        self.gamepad_data_ = None
        self.track_data_ = None

    def callback_gamepad_commands(self, gamepad_cmd):
        # Save gamepad data
        self.gamepad_data_ = gamepad_cmd

    def callback_toggle_tracking(self, track_cmd):
        # Save track data
        self.track_data_ = track_cmd

    def final_commands_publisher(self):
        # Send final commands to microcontroller, a combination of gamepad and tracking data
        final_cmd = Twist()
        
        if self.gamepad_data_ is not None and self.track_data_ is not None:
            if self.gamepad_data_.tracking_toggle == False:
                #self.get_logger().info("Tracking is disabled")
                final_cmd.linear.x = self.gamepad_data_.inclination
                final_cmd.linear.y = self.gamepad_data_.yaw_vel
                final_cmd.linear.z = self.gamepad_data_.eq_change
                final_cmd.angular.x = self.gamepad_data_.kp_change
                final_cmd.angular.y = self.gamepad_data_.ki_change
                final_cmd.angular.z = self.gamepad_data_.kd_change
                self.publisher_.publish(final_cmd)
            else:
                #self.get_logger().info("Tracking is enabled")
                final_cmd.linear.x = self.track_data_.inclination
                final_cmd.linear.y = self.track_data_.yaw_vel
                final_cmd.linear.z = self.gamepad_data_.eq_change
                final_cmd.angular.x = self.gamepad_data_.kp_change
                final_cmd.angular.y = self.gamepad_data_.ki_change
                final_cmd.angular.z = self.gamepad_data_.kd_change
                self.publisher_.publish(final_cmd)

        else:
                final_cmd.linear.x = 0.0
                final_cmd.linear.y = 0.0
                final_cmd.linear.z = 0.0
                final_cmd.angular.x = 0.0
                final_cmd.angular.y = 0.0
                final_cmd.angular.z = 0.0
                self.publisher_.publish(final_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Combiner() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()