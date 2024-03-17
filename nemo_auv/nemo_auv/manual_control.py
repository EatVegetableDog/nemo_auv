"""Node for controlling the depth of the auv.

Returns:
    _type_: _description_
"""

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from math import isclose
from nemo_auv_interfaces.srv import AuvPose
import math
from std_msgs.msg import Float64
from nemo_auv_interfaces.msg import Depth
import sys
import time
import inputs


class ManualControl(Node):
    """
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('manual_control')

        # ===== Set up Mavlink Comms ===== #
        # Create the connection to the top-side computer as companion computer/autopilot
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')

        # Wait a heartbeat before sending commands
        self.master.wait_heartbeat()

        # Initialize mode
        # self.set_mode()

        self.arm()

    def timer_callback(self):
        self.timer_cnt += 1

        read_flag = True

        while read_flag:
            msg = self.master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                read_flag = False

        # Update current position everytime you read depth
        self.depth_msg.z = msg.to_dict()["relative_alt"]

        # self.depth_pub.publish(self.depth_msg)

        self.set_rc_channel_pwm(2, 1500)
        self.set_rc_channel_pwm(3, 1500)
        self.set_rc_channel_pwm(4, 1500)
        self.set_rc_channel_pwm(5, 1500)

    def arm(self):
        """Arm the robot.
        """
        # Send message to arm vehicle
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0,
            0)

        # Confirm that vehicle is armed
        print("Waiting for the vehicle to arm")
        self.master.motors_armed_wait()
        self.get_logger().info(f'ARMED!')

    def set_mode(self, mode="MANUAL"):
        """Set the mode of the robot (i.e. Manual, Stabilize, etc). Modes are standard Ardusub modes

        Args:
            mode (str, optional): Mode you want to set the vehicle to. Defaults to "MANUAL".
        """
        self.get_logger().error(
            f"ArduSub mode = {mode}!!!!!!!!!!!!!!!!!!!!!!!!!")

        # Check if mode is available
        if mode not in self.master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.master.mode_mapping().keys()))
            sys.exit(1)

        # Get mode ID
        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.master.mav.rc_channels_override_send(self.master.target_system,
                                                  self.master.target_component,
                                                  *rc_channel_values)





def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    manual_control_node = ManualControl()

    # Spin the node so the callback function is called.
    rclpy.spin(manual_control_node)

    # Destroy the node explicitly
    manual_control_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
