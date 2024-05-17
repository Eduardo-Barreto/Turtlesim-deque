import argparse
import rclpy
from std_msgs.msg import String

parser = argparse.ArgumentParser(
    prog="Turtlesim-deque",
    description="Controla a tartaruguinha :p",
    epilog="Prova 1, Engenharia de Computação Módulo 6",
)

parser.add_argument("vx", default=0)
parser.add_argument("vy", default=0)
parser.add_argument("vt", default=0)
parser.add_argument("t", default=0)

args = parser.parse_args()


command = f"{args.vx} {args.vy} {args.vy} {args.t}"

rclpy.init()
node = rclpy.create_node("cli_node")
publisher = node.create_publisher(String, "/command", 10)
msg = String()
msg.data = command
publisher.publish(msg)
node.destroy_node()
rclpy.shutdown()
