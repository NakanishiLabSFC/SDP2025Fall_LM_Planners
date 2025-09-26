#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    
    node = Node('sequence_test')
    publisher = node.create_publisher(String, '/robot_command_sequence', 10)
    
    # Create test sequence as comma-separated string
    STEPS = [
        "ミャクミャクの椅子へ向かう",
        "3Dプリンターへ向かう", 
        "抹茶の箱へ向かう",
        "ミャクミャクの椅子へ向かう",
        "3Dプリンターへ向かう",
        "抹茶の箱へ向かう",
    ]
    
    # Create comma-separated string
    sequence_string = ", ".join(STEPS)
    
    # Wait for publisher to be ready
    node.get_logger().info("Waiting for target_nav to be ready...")
    import time
    time.sleep(2)
    
    # Create message
    msg = String()
    msg.data = sequence_string
    
    node.get_logger().info(f"Sending sequence with {len(STEPS)} actions:")
    for i, step in enumerate(STEPS):
        node.get_logger().info(f"  {i+1}. {step}")
    
    node.get_logger().info(f"Sequence string: '{sequence_string}'")
    
    # Publish sequence
    publisher.publish(msg)
    node.get_logger().info("Sequence published!")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()