#!/usr/bin/env python3
"""
Interactive CLI Drone Controller for MicroSim

Real-time keyboard controls for flying the drone.
Movement occurs ONLY while keys are held down - release to stop!

Controls:
  Movement (hold keys):
    W/S - Forward/Backward
    A/D - Left/Right
    Q/E - Rotate Left/Right
    R/F - Up/Down

  Speed:
    +/- - Increase/Decrease speed

  Quick Actions:
    SPACE - Emergency stop
    0     - Reset simulation

  Exit:
    ESC or Ctrl+C - Exit controller
"""

import sys
import termios
import tty
import select
import time
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


# ANSI color codes for pretty output
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publisher for drone velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10)

        # Service client for reset
        self.reset_client = self.create_client(Empty, '/sim/reset')

        # Control state
        self.speed_scale = 1.0  # Speed multiplier
        self.base_linear_speed = 2.0  # m/s
        self.base_angular_speed = 1.0  # rad/s

        # Track which keys are currently pressed with timestamps
        # Keys expire after KEY_TIMEOUT if not refreshed
        self.keys_pressed = {}  # key -> last_press_time
        self.KEY_TIMEOUT = 0.15  # seconds - keys expire if not re-pressed

        # Current velocity command
        self.twist = Twist()

        # Timer for publishing commands at 30 Hz (faster for responsiveness)
        self.timer = self.create_timer(1.0/30.0, self.publish_command)

        self.get_logger().info('Drone controller started!')

    def publish_command(self):
        """Publish current velocity command based on pressed keys"""
        # Remove expired keys (not pressed recently)
        current_time = time.time()
        expired_keys = [k for k, t in self.keys_pressed.items()
                       if current_time - t > self.KEY_TIMEOUT]
        for k in expired_keys:
            del self.keys_pressed[k]

        # Reset velocities
        self.twist = Twist()

        # Apply velocities based on currently pressed keys
        if 'w' in self.keys_pressed:
            self.twist.linear.x += self.base_linear_speed * self.speed_scale
        if 's' in self.keys_pressed:
            self.twist.linear.x -= self.base_linear_speed * self.speed_scale
        if 'a' in self.keys_pressed:
            self.twist.linear.y += self.base_linear_speed * self.speed_scale
        if 'd' in self.keys_pressed:
            self.twist.linear.y -= self.base_linear_speed * self.speed_scale
        if 'r' in self.keys_pressed:
            self.twist.linear.z += self.base_linear_speed * self.speed_scale
        if 'f' in self.keys_pressed:
            self.twist.linear.z -= self.base_linear_speed * self.speed_scale
        if 'q' in self.keys_pressed:
            self.twist.angular.z += self.base_angular_speed * self.speed_scale
        if 'e' in self.keys_pressed:
            self.twist.angular.z -= self.base_angular_speed * self.speed_scale

        # Publish the command
        self.cmd_pub.publish(self.twist)

    def press_key(self, key):
        """Mark a key as pressed (or refresh its timestamp)"""
        self.keys_pressed[key] = time.time()

    def reset_sim(self):
        """Reset the simulation"""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset service not available')
            return
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        self.get_logger().info('Simulation reset!')


def get_key_nonblocking(timeout=0.01):
    """
    Get a single keypress from stdin without blocking.
    Returns the key character or None if no key is available.
    """
    # Check if input is available
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def clear_screen():
    """Clear the terminal screen using OS command"""
    os.system('clear' if os.name != 'nt' else 'cls')


def print_status(controller):
    """Print current control status"""
    twist = controller.twist
    speed = controller.speed_scale
    keys = controller.keys_pressed

    # Move cursor to home and clear screen
    print('\033[H\033[2J', end='', flush=True)

    # Header
    print(f"{Colors.BOLD}{Colors.CYAN}╔══════════════════════════════════════════════════════════════╗{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}║     MicroSim Drone Controller - HOLD KEYS TO MOVE!          ║{Colors.END}")
    print(f"{Colors.BOLD}{Colors.CYAN}╚══════════════════════════════════════════════════════════════╝{Colors.END}")
    print()

    # Current velocity
    print(f"{Colors.BOLD}Current Velocity:{Colors.END}")
    print(f"  Forward/Back:  {Colors.GREEN if abs(twist.linear.x) > 0 else Colors.END}{twist.linear.x:+6.2f} m/s{Colors.END}")
    print(f"  Left/Right:    {Colors.GREEN if abs(twist.linear.y) > 0 else Colors.END}{twist.linear.y:+6.2f} m/s{Colors.END}")
    print(f"  Up/Down:       {Colors.GREEN if abs(twist.linear.z) > 0 else Colors.END}{twist.linear.z:+6.2f} m/s{Colors.END}")
    print(f"  Rotation:      {Colors.GREEN if abs(twist.angular.z) > 0 else Colors.END}{twist.angular.z:+6.2f} rad/s{Colors.END}")
    print(f"  Speed Scale:   {Colors.YELLOW}{speed:.1f}x{Colors.END}")
    print()

    # Active keys
    if keys:
        keys_str = ', '.join(sorted(keys.keys()))
        print(f"{Colors.BOLD}Keys Active:{Colors.END} {Colors.GREEN}{keys_str.upper()}{Colors.END}")
    else:
        print(f"{Colors.BOLD}Keys Active:{Colors.END} {Colors.YELLOW}(none - hold keys to move){Colors.END}")
    print()

    # Controls
    print(f"{Colors.BOLD}Controls (HOLD to move, RELEASE to stop):{Colors.END}")
    print(f"  {Colors.CYAN}Movement:{Colors.END}           {Colors.CYAN}Speed:{Colors.END}              {Colors.CYAN}Actions:{Colors.END}")
    print(f"    W/S - Fwd/Back         +/- Speed Up/Down      SPACE - Emergency Stop")
    print(f"    A/D - Left/Right                              0     - Reset Sim")
    print(f"    Q/E - Turn L/R")
    print(f"    R/F - Up/Down")
    print()
    print(f"  {Colors.RED}ESC or Ctrl+C - Exit{Colors.END}")
    print()

    # Visual indicator
    if abs(twist.linear.x) > 0 or abs(twist.linear.y) > 0 or abs(twist.linear.z) > 0 or abs(twist.angular.z) > 0:
        print(f"  {Colors.GREEN}{Colors.BOLD}✈ MOVING{Colors.END}")
    else:
        print(f"  {Colors.YELLOW}⏸ STOPPED (hold keys to move){Colors.END}")


def main():
    # Set up terminal for raw input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # Use cbreak mode instead of raw mode for better ANSI support
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init()
        controller = DroneController()

        # Clear screen and show startup message
        clear_screen()
        print(f"\n{Colors.BOLD}{Colors.GREEN}Drone Controller Started!{Colors.END}")
        print(f"{Colors.YELLOW}HOLD keys to move - release to stop! Press ESC to exit.{Colors.END}\n")
        time.sleep(1)

        print_status(controller)
        last_display_update = time.time()
        display_update_interval = 0.1  # Update display at most 10 times per second

        while True:
            # Spin ROS for callbacks
            rclpy.spin_once(controller, timeout_sec=0)

            # Check for key input (non-blocking with short timeout)
            key = get_key_nonblocking(timeout=0.01)

            should_update_display = False

            if key:
                # Exit conditions
                if key == '\x1b':  # ESC
                    break
                elif key == '\x03':  # Ctrl+C
                    break

                # Movement keys - register press with timestamp
                elif key in 'wsadqerf':
                    controller.press_key(key)
                    should_update_display = True

                # Speed controls - instant actions
                elif key in '+=':
                    controller.speed_scale = min(3.0, controller.speed_scale + 0.2)
                    should_update_display = True
                elif key in '-_':
                    controller.speed_scale = max(0.2, controller.speed_scale - 0.2)
                    should_update_display = True

                # Quick actions
                elif key == ' ':  # Space - emergency stop
                    controller.keys_pressed.clear()
                    should_update_display = True
                elif key == '0':  # Reset
                    controller.reset_sim()
                    controller.keys_pressed.clear()
                    should_update_display = True

            # Update display periodically or on input
            current_time = time.time()
            if should_update_display or (current_time - last_display_update > display_update_interval):
                print_status(controller)
                last_display_update = current_time

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        # Clear screen and stop drone
        clear_screen()
        controller.keys_pressed.clear()
        controller.twist = Twist()
        controller.cmd_pub.publish(controller.twist)

        print(f"\n{Colors.YELLOW}Controller stopped. Drone stopped.{Colors.END}\n")
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
