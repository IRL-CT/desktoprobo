import math
import subprocess
import rclpy
import tf2_ros
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from std_msgs.msg import *
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, TransformStamped

# XL330 Control Table (Protocol 2.0)
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132

# U2D2 / bus settings
PROTOCOL_VERSION = 2.0
DEVICE_NAME      = '/dev/ttyUSB0'
BAUDRATE         = 115200

# Motor IDs (match your physical setup)
DXL_ID_LEFT  = 1   # axis0 equivalent
DXL_ID_RIGHT = 2   # axis1 equivalent

VELOCITY_MODE  = 1
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0
VEL_UNIT_RPM   = 0.229   # rpm per Dynamixel velocity unit


class odrive_command(Node):
    def __init__(self):
        super().__init__("command_lisener")

        # --- Latency timer (must be 1 ms for reliable comms) ---
        try:
            dev = DEVICE_NAME.split('/')[-1]  # e.g. ttyUSB0
            subprocess.run(
                ['sudo', 'bash', '-c', f'echo 1 > /sys/bus/usb-serial/devices/{dev}/latency_timer'],
                check=True)
            self.get_logger().info(f"Latency timer set to 1ms for {dev}")
        except Exception as e:
            self.get_logger().warn(f"Could not set latency timer: {e}")

        # --- U2D2 / Dynamixel initialisation ---
        self.portHandler   = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open U2D2 port")
            raise RuntimeError("Failed to open U2D2 port")

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baud rate")
            raise RuntimeError("Failed to set baud rate")

        self.active_motors = []
        for dxl_id in [DXL_ID_LEFT, DXL_ID_RIGHT]:
            _, result, _ = self.packetHandler.ping(self.portHandler, dxl_id)
            if result != COMM_SUCCESS:
                self.get_logger().warn(f"Motor ID {dxl_id} not found, skipping")
                continue
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_OPERATING_MODE, VELOCITY_MODE)
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            self.active_motors.append(dxl_id)
            self.get_logger().info(f"Motor ID {dxl_id} initialized OK")

        if not self.active_motors:
            self.get_logger().error("No motors found! Check wiring and baud rate.")

        self.get_logger().info("U2D2/XL330 initialization done")

        self.old_pos_l = 0
        self.old_pos_r = 0
        timer_period = 0.1  # seconds
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_timer = self.create_timer(timer_period, self.odom_timer_callback)

        # setup message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"
        self.encoder_cpr = 4096   # XL330 position resolution (counts/rev)
        self.odom_calc_hz = 10

        # store current location to be updated.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # TODO: Robot Wheel Specifications -------------------------------
        # self.R is wheel radius (in meters)
        self.R = 0.023  # 4.6 cm diameter → 2.3 cm radius
        self.tyre_circumference = 2 * math.pi * self.R
        self.wheel_track = 0.135  # 13.5 cm
        # ----------------------------------------------------------------

        self.tf_publisher = tf2_ros.TransformBroadcaster(self)
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "odom"
        self.tf_msg.child_frame_id  = "base_link"

        self.twist_sub = self.create_subscription(Twist, "/cmd_vel", self.diff_drive_callback, 10)
        self.get_logger().info("ros initialization done")

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------

    def _write_velocity(self, dxl_id, vel_unit):
        """Write a signed velocity (Dynamixel units) to a motor."""
        val = int(vel_unit) & 0xFFFFFFFF          # two's-complement → uint32
        self.packetHandler.write4ByteTxRx(
            self.portHandler, dxl_id, ADDR_GOAL_VELOCITY, val)

    @staticmethod
    def _to_signed32(raw):
        return raw - 0x100000000 if raw > 0x7FFFFFFF else raw

    def _read_velocity(self, dxl_id):
        """Return present velocity in Dynamixel units (signed)."""
        raw, _, _ = self.packetHandler.read4ByteTxRx(
            self.portHandler, dxl_id, ADDR_PRESENT_VELOCITY)
        return self._to_signed32(raw)

    def _read_position(self, dxl_id):
        """Return present position in counts (signed)."""
        raw, _, _ = self.packetHandler.read4ByteTxRx(
            self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
        return self._to_signed32(raw)

    # ------------------------------------------------------------------

    def diff_drive_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        # Differential drive inverse kinematics
        # v  = R * (Vl + Vr) / 2          → linear velocity
        # w  = R * (Vr - Vl) / wheel_track → angular velocity
        # Solving for wheel angular speeds (rad/s):
        Vl = (v - w * self.wheel_track / 2.0) / self.R
        Vr = (v + w * self.wheel_track / 2.0) / self.R

        # convert rad/s → Dynamixel velocity units (0.229 rpm/unit)
        rad_s_to_unit = (60.0 / (2 * math.pi)) / VEL_UNIT_RPM

        if abs(Vl) <= 0.08:
            self._write_velocity(DXL_ID_LEFT, 0)
        else:
            self._write_velocity(DXL_ID_LEFT, int(Vl * rad_s_to_unit))

        if abs(Vr) <= 0.08:
            self._write_velocity(DXL_ID_RIGHT, 0)
        else:
            self._write_velocity(DXL_ID_RIGHT, int(-Vr * rad_s_to_unit))

######################################### ODOMETRY #################################

    def odom_timer_callback(self):
        time_now = self.get_clock().now()
        self.vel_l, self.vel_r = 0, 0
        self.new_pos_l, self.new_pos_r = 0, 0
        self.m_s_to_value = self.encoder_cpr / self.tyre_circumference
        # Convert Dynamixel velocity units → counts/sec (same unit as ODrive encoder)
        unit_to_counts_s = (VEL_UNIT_RPM / 60.0) * self.encoder_cpr
        try:
            self.vel_l =  self._read_velocity(DXL_ID_LEFT)  * unit_to_counts_s
            self.vel_r = -self._read_velocity(DXL_ID_RIGHT) * unit_to_counts_s  # neg = forward
            self.new_pos_l =  self._read_position(DXL_ID_LEFT)
            self.new_pos_r = -self._read_position(DXL_ID_RIGHT)
            self.pub_odometry(time_now)
        except Exception as e:
            self.get_logger().info(str(e))
            pass

    def pub_odometry(self, time_now):
        now_stamp = time_now.to_msg()
        self.odom_msg.header.stamp = now_stamp
        self.tf_msg.header.stamp = now_stamp
        # Twist/velocity: calculated from motor values only
        s = self.tyre_circumference * (self.vel_l + self.vel_r) / (2.0 * self.encoder_cpr)
        w = self.tyre_circumference * (self.vel_r - self.vel_l) / (self.wheel_track * self.encoder_cpr)
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w

        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r

        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r

        # Check for overflow. Assume we can't move more than half a circumference in a single timestep.
        half_cpr = self.encoder_cpr / 2.0
        if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - self.encoder_cpr
        elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + self.encoder_cpr
        if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - self.encoder_cpr
        elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + self.encoder_cpr

        # counts to metres
        delta_pos_l_m = delta_pos_l / self.m_s_to_value
        delta_pos_r_m = delta_pos_r / self.m_s_to_value

        # Distance travelled
        d  = (delta_pos_l_m + delta_pos_r_m) / 2.0
        th = (delta_pos_r_m - delta_pos_l_m) / self.wheel_track

        xd = math.cos(th) * d
        yd = -math.sin(th) * d

        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta) * xd - math.sin(self.theta) * yd
        self.y += math.sin(self.theta) * xd + math.cos(self.theta) * yd
        self.theta = (self.theta + th) % (2 * math.pi)

        # fill odom message and publish
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]

        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]

        self.odom_publisher.publish(self.odom_msg)
        self.tf_publisher.sendTransform(self.tf_msg)

    def shutdown(self):
        for dxl_id in self.active_motors:
            self._write_velocity(dxl_id, 0)
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()
