#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from time import sleep

def talker_main():
	rclpy.init(args=None)
	node = rclpy.create_node('ros2_talker_node')
	pub = node.create_publisher(String, '/chatter')
	msg = String()
	i = 0
	while rclpy.ok():
		msg.data = 'Hello World: %d' % i
		i += 1
		node.get_logger().info('Publishing: "%s"' % msg.data)
		pub.publish(msg)
		sleep(0.5)

if __name__ == '__main__':
	talker_main()
