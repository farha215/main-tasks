#!/usr/bin/env python3
import rclpy
import sys, select, tty, termios, signal, time
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


# ================= PID Controller =================
class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max, integral_limit=200.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, setpoint, measurement):
        now = time.time()
        if self._prev_time is None:
            self._prev_time = now
            return 0.0

        dt = now - self._prev_time
        if dt <= 0.0:
            return 0.0
        self._prev_time = now

        error = setpoint - measurement
        self._integral = clamp(
            self._integral + error * dt,
            -self.integral_limit, self.integral_limit
        )
        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        return clamp(output, self.output_min, self.output_max)

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_thrusters")

        # ================= Controller Subscriptions =================
        self.sub_front = self.create_subscription(
            Float64, 'new_thrust_front', self.cb_front, 10)
        self.sub_left = self.create_subscription(
            Float64, 'new_thrust_left', self.cb_left, 10)
        self.sub_right = self.create_subscription(
            Float64, 'new_thrust_right', self.cb_right, 10)

        # ================= Odometry for depth feedback =================
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.cb_odom, 10)
        self.current_depth = 0.0
        self.odom_received = False

        # ================= Thruster Publishers =================
        self.pubs = {
            'back_propeller':    self.create_publisher(Float64, '/hydrogen/back_propeller/cmd_thrust', 10),
            'right_propeller_1': self.create_publisher(Float64, '/hydrogen/right_propeller_1/cmd_thrust', 10),
            'right_propeller_2': self.create_publisher(Float64, '/hydrogen/right_propeller_2/cmd_thrust', 10),
            'left_propeller_1':  self.create_publisher(Float64, '/hydrogen/left_propeller_1/cmd_thrust', 10),
            'left_propeller_2':  self.create_publisher(Float64, '/hydrogen/left_propeller_2/cmd_thrust', 10),
        }

        # ================= Controller Values =================
        self.ctrl_values = {
            'back_propeller':    0.0,
            'left_propeller_2':  0.0,
            'right_propeller_2': 0.0,
        }

        self.manual_offsets = {k: 0.0 for k in self.pubs.keys()}

        self.step = 50.0
        self.max_thrust = 50.0

        # ================= Depth PID ? always active once odom arrives =================
        self.depth_pid = PIDController(
            kp=20.0,
            ki=2.0,
            kd=5.0,
            output_min=-100.0,
            output_max=100.0,
            integral_limit=150.0
        )
        self.target_depth = 0.0
        self.depth_step   = 0.1       # metres per keypress

        # While A/D held: manual override replaces PID output
        # When released: PID snaps setpoint to current depth and resumes
        self.manual_vertical = 0.0    # non-zero only while A/D pressed
        self.vertical_step   = 5.0
        self.vertical_max    = 100.0

        self.timer = self.create_timer(0.02, self.publish_all)

    # ================= Callbacks =================
    def cb_front(self, msg):
        self.ctrl_values['back_propeller'] = msg.data

    def cb_left(self, msg):
        self.ctrl_values['left_propeller_2'] = msg.data

    def cb_right(self, msg):
        self.ctrl_values['right_propeller_2'] = msg.data

    def cb_odom(self, msg):
        self.current_depth = -msg.pose.pose.position.z
        if not self.odom_received:
            # First odom ? initialise hold at current depth
            self.target_depth = self.current_depth
            self.odom_received = True
            self.get_logger().info(f"Odom received. Holding depth {self.target_depth:.2f} m")

    # ================= Publishing =================
    def publish_all(self):
        if not self.odom_received:
            # No odom yet ? publish zeros and wait
            for pub in self.pubs.values():
                msg = Float64()
                msg.data = 0.0
                pub.publish(msg)
            return

        if self.manual_vertical != 0.0:
            # A/D pressed: bypass PID, drive manually
            vert = self.manual_vertical
        else:
            # PID holds target depth automatically
            vert = self.depth_pid.compute(self.target_depth, self.current_depth)

        for name, pub in self.pubs.items():
            ctrl = self.ctrl_values.get(name, 0.0)

            if name in ('left_propeller_2', 'right_propeller_2'):
                blended = ctrl + self.manual_offsets[name] + vert
            elif name == 'back_propeller':
                blended = ctrl + self.manual_offsets[name] + (vert * 0.9)
            else:
                blended = ctrl + self.manual_offsets[name]

            msg = Float64()
            msg.data = float(blended)
            pub.publish(msg)

    def stop_all(self):
        for k in self.manual_offsets:
            self.manual_offsets[k] = 0.0
        self.manual_vertical = 0.0
        # Snap hold to current depth and reset PID
        self.target_depth = self.current_depth
        self.depth_pid.reset()
        self.publish_all()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print("""
Teleop + Automatic Depth Hold

Depth hold is ALWAYS active. The AUV holds its current depth
automatically. Use A/D to ascend/descend ? depth hold resumes
at the new depth the moment you release.

  A               : Ascend  (manual override while held)
  D               : Descend (manual override while held)
  Arrow Left/Right: Yaw
  W / S           : Forward / Backward
  I / K           : Pitch
  L / R           : Roll left / right
  SPACE           : Stop all + re-lock depth at current position
  X               : Exit
""")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def exit_clean(*_):
        node.stop_all()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_clean)

    try:
        while rclpy.ok():
            # Reset horizontal offsets every cycle (they are instantaneous)
            for k in node.manual_offsets:
                node.manual_offsets[k] = 0.0

            # Reset manual vertical every cycle ? A/D must be held to keep override
            prev_manual_vertical = node.manual_vertical
            node.manual_vertical = 0.0

            if is_data():
                ch = sys.stdin.read(1)

                if ch == '\x1b':
                    seq = sys.stdin.read(2)

                    if seq == '[C':    # yaw right
                        node.manual_offsets['left_propeller_1']  =  node.step
                        node.manual_offsets['right_propeller_1'] = -node.step

                    elif seq == '[D':  # yaw left
                        node.manual_offsets['left_propeller_1']  = -node.step
                        node.manual_offsets['right_propeller_1'] =  node.step

                else:
                    ch = ch.lower()

                    if ch == 'w':
                        node.manual_offsets['left_propeller_1']  = -node.step
                        node.manual_offsets['right_propeller_1'] = -node.step

                    elif ch == 's':
                        node.manual_offsets['left_propeller_1']  =  node.step
                        node.manual_offsets['right_propeller_1'] =  node.step

                    elif ch == 'a':   # ascend ? manual override
                        node.manual_vertical = clamp(
                            node.vertical_step,
                            -node.vertical_max, node.vertical_max)

                    elif ch == 'd':   # descend ? manual override
                        node.manual_vertical = clamp(
                            -node.vertical_step,
                            -node.vertical_max, node.vertical_max)

                    elif ch == 'i':
                        node.manual_offsets['left_propeller_2']  =  node.step
                        node.manual_offsets['right_propeller_2'] = -node.step

                    elif ch == 'k':
                        node.manual_offsets['left_propeller_2']  = -node.step
                        node.manual_offsets['right_propeller_2'] =  node.step

                    elif ch == 'l':
                        node.manual_offsets['left_propeller_2']  = -node.step
                        node.manual_offsets['right_propeller_2'] =  node.step

                    elif ch == 'r':
                        node.manual_offsets['left_propeller_2']  =  node.step
                        node.manual_offsets['right_propeller_2'] = -node.step

                    elif ch == ' ':
                        node.stop_all()
                        print(f"  Stopped. Holding depth {node.target_depth:.2f} m")

                    elif ch == 'x':
                        exit_clean()

            # A/D just released ? snap PID setpoint to current depth
            if prev_manual_vertical != 0.0 and node.manual_vertical == 0.0:
                node.target_depth = node.current_depth
                node.depth_pid.reset()
                print(f"  Depth hold re-locked at {node.target_depth:.2f} m")

            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        exit_clean()


if __name__ == "__main__":
    main()