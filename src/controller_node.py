import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from collections import deque


class TeleopService(Node):
    def __init__(self):
        """
        Initialize the teleop service node.
        """
        super().__init__("teleop_service")

        self.current_command = ""

        self.is_killed = False
        self.command_queue = deque()

        self.command_subscriber = self.create_subscription(
            String, "command", self.command_callback, 10
        )

        self.kill_service = self.create_service(Empty, "kill_robot", self.kill_callback)

        self.cmd_vel_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.timer = self.create_timer(1, self.timer_callback)

        self.get_logger().info("Teleop service node initialized.")

    def kill_callback(self, request, response):
        """
        Callback to terminate the teleop service node.
        """
        self.get_logger().info("Teleop service node terminated.")
        self.is_killed = True
        return response

    def command_callback(self, msg):
        """
        Callback to receive a new command
        """
        vx, vy, vtheta, time = msg.data.split(" ")
        command = {
            "vx": float(vx),
            "vy": float(vy),
            "vtheta": float(vtheta),
            "time": float(time)/1000,
        }
        print(f"Command received: {msg.data}")
        self.command_queue.append(command)

    def timer_callback(self):
        """
        Callback to timer
        """
        command = Twist()
        current = None

        if (len(self.command_queue)) != 0:
            current = self.command_queue.pop()
            command.linear.x = current.get("vx")
            command.linear.y = current.get("vy")
            command.angular.z = current.get("vtheta")

            print(f"Executing command: {command}")
            self.timer.destroy()
            self.timer = self.create_timer(
                current.get("time"), self.timer_callback
            )
            self.cmd_vel_publisher.publish(command)
        else:
            pass

    def spin(self):
        """
        Runs the teleop service node.
        """
        try:
            while rclpy.ok() and not self.is_killed:
                rclpy.spin_once(self)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

        finally:
            self.stop_robot()
            self.get_logger().info("Teleop service node terminated.")

    def stop_robot(self):
        """
        Stops the robot's movement.
        """
        self.current_speed = Twist()
        self.cmd_vel_publisher.publish(self.current_speed)
        self.get_logger().info("Robot movement stopped.")


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopService()
    teleop_node.spin()
    teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
