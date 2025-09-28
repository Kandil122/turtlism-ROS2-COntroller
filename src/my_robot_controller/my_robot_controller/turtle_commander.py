#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')

        # State
        self.shape_code = 0
        self.drawing = False
        self.t = 0.0                   # parameter for parametric curves
        self.t_step = 0.1              # how much t increases each timer tick (radians)
        self.start_x = 0.0
        self.start_y = 0.0
        self.curr_target_index = 0     # for vertex-based shapes
        self.targets = []              # list of (x,y) for polygon/star/trapezium

        # Pose feedback
        self.pose = None
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Shape command subscription (Int32 on /shape)
        self.shape_sub = self.create_subscription(Int32, '/shape', self.shape_callback, 10)

        # Velocity publisher
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Timer for control loop
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Controller gains and limits (tune these if shape looks off)
        self.Kp_lin = 1.2
        self.Kp_ang = 6.0
        self.max_lin = 2.0
        self.max_ang = 6.0
        self.angle_threshold_for_forward = 0.6  # rad; if heading error > this, slow forward

        self.get_logger().info("TurtleCommander ready. Waiting for /shape (Int32).")

    # -------------------
    # ROS callbacks
    # -------------------
    def pose_callback(self, msg: Pose):
        self.pose = msg

    def shape_callback(self, msg: Int32):
        code = int(msg.data)
        self.get_logger().info(f"Received shape command: {code}")

        if self.pose is None:
            self.get_logger().warn("No current pose yet — ignoring shape command until pose is available.")
            return

        if code == 4:  # stop
            self.get_logger().info("Stop command received. Stopping turtle and clearing shape.")
            self.stop_and_reset()
            return

        # Save start center so shape is drawn relative to current pose
        self.start_x = self.pose.x
        self.start_y = self.pose.y

        # Setup shape-specific parameters
        if code == 1:
            # Heart (parametric Cartesian form)
            self.shape_code = 1
            self.t = 0.0
            self.t_max = 2.0 * math.pi
            self.scale = 5.0
            self.drawing = True

        elif code == 2:
            # Spiral (Archimedean): r = a + b*theta
            self.shape_code = 2
            self.t = 0.0
            self.t_max = 6.0 * math.pi
            self.spiral_a = 0.05
            self.spiral_b = 0.15
            self.scale = 0.9
            self.drawing = True

        elif code == 3:
            # Star — use vertex list
            self.shape_code = 3
            self.drawing = True
            self.curr_target_index = 0
            self.targets = self._make_star_vertices(
                n_points=5,
                outer_r=1.6,
                inner_r=0.6,
                center=(self.start_x, self.start_y)
            )
            self.targets.append(self.targets[0])  # close shape

        elif code == 5:
            # Trapezium — vertex list
            self.shape_code = 3
            self.drawing = True
            self.curr_target_index = 0
            top_w = 2.0
            bottom_w = 4.0
            height = 2.0
            cx, cy = self.start_x, self.start_y
            self.targets = [
                (cx - bottom_w/2, cy - height/2),  # bottom-left
                (cx + bottom_w/2, cy - height/2),  # bottom-right
                (cx + top_w/2,    cy + height/2),  # top-right
                (cx - top_w/2,    cy + height/2),  # top-left
                (cx - bottom_w/2, cy - height/2)   # back to bottom-left
            ]

        else:
            self.get_logger().warn(f"Unknown shape code {code}. Ignoring.")
            return

        self.get_logger().info(f"Starting drawing shape {code} at ({self.start_x:.2f},{self.start_y:.2f})")

    # -------------------
    # Utilities
    # -------------------
    def _make_star_vertices(self, n_points=5, outer_r=1.6, inner_r=0.6, center=(0.0,0.0)):
        cx, cy = center
        pts = []
        for i in range(n_points):
            ang_outer = 2.0 * math.pi * i / n_points
            ang_inner = ang_outer + math.pi / n_points
            ox = cx + outer_r * math.cos(ang_outer)
            oy = cy + outer_r * math.sin(ang_outer)
            ix = cx + inner_r * math.cos(ang_inner)
            iy = cy + inner_r * math.sin(ang_inner)
            pts.append((ox, oy))
            pts.append((ix, iy))
        return pts

    def _polar_to_xy(self, r, theta_rad):
        x = r * math.cos(theta_rad)
        y = r * math.sin(theta_rad)
        return x, y

    # -------------------
    # Control loop
    # -------------------
    def control_loop(self):
        if not self.drawing:
            return
        if self.pose is None:
            return

        if self.shape_code in (1, 2):
            if self.t > self.t_max:
                self.get_logger().info("Finished parametric shape.")
                self.stop_and_reset()
                return

            if self.shape_code == 1:
                # Heart curve
                s = 0.025
                heart_step = 0.06
                xh = 16.0 * (math.sin(self.t) ** 3)
                yh = 13.0 * math.cos(self.t) - 5.0 * math.cos(2.0 * self.t) - 2.0 * math.cos(3.0 * self.t) - math.cos(4.0 * self.t)
                sx = s * xh
                sy = -s * yh
                self.t += heart_step
            else:
                r_val = self.spiral_a + self.spiral_b * self.t
                sx, sy = self._polar_to_xy(r_val * self.scale, self.t)
                self.t += self.t_step

            target_x = self.start_x + sx
            target_y = self.start_y + sy
            self._goto(target_x, target_y)

        elif self.shape_code == 3:
            if self.curr_target_index >= len(self.targets):
                self.get_logger().info("Finished vertex shape.")
                self.stop_and_reset()
                return

            tx, ty = self.targets[self.curr_target_index]
            dist = math.hypot(tx - self.pose.x, ty - self.pose.y)
            if dist < 0.12:
                self.curr_target_index += 1
                return
            self._goto(tx, ty)

    def _goto(self, target_x, target_y):
        dx = target_x - self.pose.x
        dy = target_y - self.pose.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_err = normalize_angle(angle_to_target - self.pose.theta)

        ang = clamp(self.Kp_ang * angle_err, -self.max_ang, self.max_ang)
        if abs(angle_err) > self.angle_threshold_for_forward:
            lin = 0.15
        else:
            lin = clamp(self.Kp_lin * dist, 0.0, self.max_lin)

        if dist < 0.03:
            lin = 0.0
            ang = 0.0

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_pub.publish(cmd)

    def stop_and_reset(self):
        for _ in range(3):
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

        self.drawing = False
        self.shape_code = 0
        self.t = 0.0
        self.curr_target_index = 0
        self.targets = []

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
