#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from .utils import create_pose_stamped, extract_pose_info


class TargetNavNode(Node):
    def __init__(self):
        super().__init__('target_nav')
        
        # Declare parameters
        self.declare_parameter('object_map_path', 'config/object_map.yaml')
        self.declare_parameter('goal_timeout_sec', 120.0)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('approach_offset', 0.0)
        
        # Get parameters
        self.object_map_path = self.get_parameter('object_map_path').get_parameter_value().string_value
        self.goal_timeout_sec = self.get_parameter('goal_timeout_sec').get_parameter_value().double_value
        self.max_retries = self.get_parameter('max_retries').get_parameter_value().integer_value
        self.approach_offset = self.get_parameter('approach_offset').get_parameter_value().double_value
        
        # Load object map
        self.objects = self.load_object_map()
        if not self.objects:
            self.get_logger().error('Failed to load object map or object map is empty')
            return
        
        # Load command map
        self.command_map = self.load_command_map()
        
        # Create callback group for concurrent operations
        callback_group = ReentrantCallbackGroup()
        
        # Create action client for NavigateToPose
        self.nav_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=callback_group
        )
        
        # Create subscribers
        self.name_sub = self.create_subscription(
            String,
            '/go_to_name',
            self.on_name_callback,
            10,
            callback_group=callback_group
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/go_to_pose', 
            self.on_pose_callback,
            10,
            callback_group=callback_group
        )
        
        # Create robot command subscriber
        self.command_sub = self.create_subscription(
            String,
            '/robot_command_input',
            self.on_robot_command_callback,
            10,
            callback_group=callback_group
        )
        
        # Create sequence subscriber
        self.sequence_sub = self.create_subscription(
            String,
            '/robot_command_sequence',
            self.on_sequence_callback,
            10,
            callback_group=callback_group
        )
        
        # Create result publishers
        self.result_pub = self.create_publisher(String, '/nav_result', 10)
        self.sequence_status_pub = self.create_publisher(String, '/sequence_status', 10)
        
        # Track current navigation state
        self.current_goal_handle = None
        self.retry_count = 0
        self.current_tag = ""
        
        # Sequence management state
        self.action_queue = []
        self.current_action_index = 0
        self.sequence_active = False
        self.sequence_id = 0
        
        self.get_logger().info(f'Target navigation node initialized')
        self.get_logger().info(f'Loaded {len(self.objects)} objects from {self.object_map_path}')
        self.get_logger().info(f'Objects: {list(self.objects.keys())}')

    def load_object_map(self):
        """Load object coordinates from YAML file"""
        # Handle both relative and absolute paths
        if not os.path.isabs(self.object_map_path):
            package_share = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                self.object_map_path
            )
            if os.path.exists(package_share):
                yaml_path = package_share
            else:
                # Try relative to current working directory
                yaml_path = self.object_map_path
        else:
            yaml_path = self.object_map_path
            
        try:
            with open(yaml_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
                self.get_logger().info(f'Successfully loaded object map from {yaml_path}')
                return data.get('objects', {})
        except FileNotFoundError:
            self.get_logger().error(f'Object map file not found: {yaml_path}')
            return {}
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing YAML file: {e}')
            return {}
        except Exception as e:
            self.get_logger().error(f'Unexpected error loading object map: {e}')
            return {}

    def load_command_map(self):
        """Load command mapping from YAML file"""
        command_map_path = self.object_map_path.replace('object_map.yaml', 'command_map.yaml')
        
        try:
            with open(command_map_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
                self.get_logger().info(f'Successfully loaded command map from {command_map_path}')
                return data
        except FileNotFoundError:
            self.get_logger().warn(f'Command map file not found: {command_map_path}, using default mapping')
            return {
                'navigation_commands': {
                    'ミャクミャクの椅子へ向かう': 'ミャクミャクの椅子',
                    '抹茶の箱へ向かう': '抹茶の箱',
                    '3Dプリンターへ向かう': '3Dプリンター'
                },
                'unsupported_patterns': [],
                'special_commands': {'行動終了': 'END_ACTION'}
            }
        except yaml.YAMLError as e:
            self.get_logger().error(f'Error parsing command map YAML file: {e}')
            return {}
        except Exception as e:
            self.get_logger().error(f'Unexpected error loading command map: {e}')
            return {}

    def on_name_callback(self, msg):
        """Handle named target navigation request"""
        name = msg.data.strip()
        self.get_logger().info(f'Received name-based navigation request: "{name}"')
        
        if name not in self.objects:
            error_msg = f'FAILED:name={name} - object not found'
            self.get_logger().error(f'Object "{name}" not found in map')
            self.publish_result(error_msg)
            return
            
        # Get object coordinates
        obj_data = self.objects[name]
        try:
            x = float(obj_data['x'])
            y = float(obj_data['y'])
            yaw = float(obj_data['yaw'])
            frame_id = obj_data.get('frame_id', 'map')
            
            # Create pose
            pose = create_pose_stamped(x, y, yaw, frame_id)
            tag = f"name={name}"
            
            self.go_to(pose, tag)
            
        except (KeyError, ValueError, TypeError) as e:
            error_msg = f'FAILED:name={name} - invalid object data: {e}'
            self.get_logger().error(error_msg)
            self.publish_result(error_msg)

    def on_pose_callback(self, msg):
        """Handle direct pose navigation request"""
        x, y, yaw = extract_pose_info(msg)
        self.get_logger().info(f'Received pose-based navigation request: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
        
        # Validate frame_id
        if msg.header.frame_id != 'map':
            error_msg = f'FAILED:pose=({x:.3f},{y:.3f},{yaw:.3f}) - invalid frame_id: {msg.header.frame_id}'
            self.get_logger().error(f'Invalid frame_id: {msg.header.frame_id}, expected "map"')
            self.publish_result(error_msg)
            return
            
        tag = f"pose=({x:.3f},{y:.3f},{yaw:.3f})"
        self.go_to(msg, tag)

    def on_robot_command_callback(self, msg):
        """Handle high-level robot command"""
        command = msg.data.strip()
        self.get_logger().info(f'Received robot command: "{command}"')
        
        # Check if it's a navigation command
        nav_commands = self.command_map.get('navigation_commands', {})
        if command in nav_commands:
            target_name = nav_commands[command]
            self.get_logger().info(f'Processing navigation command: {command} -> {target_name}')
            
            # Use existing navigation logic
            if target_name not in self.objects:
                error_msg = f'NAVIGATION:FAILED:name={target_name} - object not found'
                self.get_logger().error(f'Object "{target_name}" not found in map')
                self.publish_result(error_msg)
                return
                
            # Get object coordinates and navigate
            obj_data = self.objects[target_name]
            try:
                x = float(obj_data['x'])
                y = float(obj_data['y'])
                yaw = float(obj_data['yaw'])
                frame_id = obj_data.get('frame_id', 'map')
                
                pose = create_pose_stamped(x, y, yaw, frame_id)
                tag = f"command={command}"
                
                self.go_to(pose, tag)
                
            except (KeyError, ValueError, TypeError) as e:
                error_msg = f'NAVIGATION:FAILED:command={command} - invalid object data: {e}'
                self.get_logger().error(error_msg)
                self.publish_result(error_msg)
            return
        
        # Check if it's a special command
        special_commands = self.command_map.get('special_commands', {})
        if command in special_commands:
            action = special_commands[command]
            self.get_logger().info(f'Processing special command: {command} -> {action}')
            self.publish_result(f'SPECIAL:{action}:command={command}')
            return
            
        # Check if it's an unsupported pattern
        unsupported_patterns = self.command_map.get('unsupported_patterns', [])
        for pattern in unsupported_patterns:
            if pattern in command:
                self.get_logger().warn(f'Unsupported command pattern detected: "{command}"')
                self.publish_result(f'UNSUPPORTED:command={command}')
                return
                
        # Unknown command
        self.get_logger().warn(f'Unknown command: "{command}"')
        self.publish_result(f'UNKNOWN:command={command}')

    def go_to(self, pose, tag):
        """Send navigation goal to Nav2"""
        self.current_tag = tag
        self.retry_count = 0
        
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('NavigateToPose action server not available')
            self.publish_result(f'FAILED:{tag} - server_unavailable')
            return
            
        self._send_goal(pose, tag)

    def _send_goal(self, pose, tag):
        """Internal method to send navigation goal"""
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f'Sending navigation goal: {tag}')
        
        # Send goal asynchronously
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            error_msg = f'FAILED:{self.current_tag} - goal rejected by server'
            self.get_logger().error('Navigation goal rejected')
            self.publish_result(error_msg)
            return
            
        self.current_goal_handle = goal_handle
        self.get_logger().info(f'Navigation goal accepted: {self.current_tag}')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional progress logging)"""
        # Extract distance remaining if available
        if hasattr(feedback_msg.feedback, 'distance_remaining'):
            distance = feedback_msg.feedback.distance_remaining
            self.get_logger().debug(f'Distance remaining: {distance:.2f}m')

    def result_callback(self, future):
        """Handle navigation result"""
        try:
            result = future.result()
            if result is None:
                self.get_logger().error('Navigation result is None')
                self.retry_or_fail('UNKNOWN')
                return
                
            status = result.status
            
            if status == 4:  # SUCCEEDED
                if self.current_tag.startswith('command='):
                    success_msg = f'NAVIGATION:SUCCEEDED:{self.current_tag}'
                else:
                    success_msg = f'SUCCEEDED:{self.current_tag}'
                self.get_logger().info(f'Navigation succeeded: {self.current_tag}')
                self.publish_result(success_msg)
                
            elif status == 2:  # CANCELED
                if self.current_tag.startswith('command='):
                    cancel_msg = f'NAVIGATION:CANCELED:{self.current_tag}'
                else:
                    cancel_msg = f'CANCELED:{self.current_tag}'
                self.get_logger().warn(f'Navigation canceled: {self.current_tag}')
                self.publish_result(cancel_msg)
                
            else:  # FAILED or other error statuses
                self.get_logger().warn(f'Navigation failed with status {status}: {self.current_tag}')
                self.retry_or_fail(str(status))
                
        except Exception as e:
            self.get_logger().error(f'Error in result callback: {e}')
            self.retry_or_fail('EXCEPTION')

    def retry_or_fail(self, error_code):
        """Handle retry logic or final failure"""
        self.retry_count += 1
        
        if self.retry_count <= self.max_retries:
            self.get_logger().info(f'Retrying navigation ({self.retry_count}/{self.max_retries}): {self.current_tag}')
            # For sequence mode, don't retry - just fail and move to next action
            if self.sequence_active:
                if self.current_tag.startswith('sequence='):
                    fail_msg = f'SEQUENCE:FAILED:{self.current_tag} code={error_code}'
                else:
                    fail_msg = f'FAILED:{self.current_tag} code={error_code}'
                self.get_logger().warn(f'Sequence navigation failed, moving to next action: {self.current_tag}')
                self.publish_result(fail_msg)
            else:
                # TODO: Implement actual retry for non-sequence navigation
                fail_msg = f'FAILED:{self.current_tag} code={error_code}'
                self.get_logger().error(f'Navigation failed, no retry implemented: {self.current_tag}')
                self.publish_result(fail_msg)
        else:
            # Max retries exceeded
            if self.current_tag.startswith('sequence='):
                fail_msg = f'SEQUENCE:FAILED:{self.current_tag} code={error_code}'
            elif self.current_tag.startswith('command='):
                fail_msg = f'NAVIGATION:FAILED:{self.current_tag} code={error_code}'
            else:
                fail_msg = f'FAILED:{self.current_tag} code={error_code}'
            self.get_logger().error(f'Navigation failed after {self.max_retries} retries: {self.current_tag}')
            self.publish_result(fail_msg)

    def publish_result(self, result_str):
        """Publish navigation result"""
        msg = String()
        msg.data = result_str
        self.result_pub.publish(msg)
        
        # Check if we're in sequence mode and handle next action
        if self.sequence_active:
            self.handle_sequence_progress(result_str)

    def on_sequence_callback(self, msg):
        """Handle sequence of comma-separated commands"""
        # Check if a sequence is already active
        if self.sequence_active:
            self.get_logger().warn(f'New sequence received while sequence {self.sequence_id} is active. Interrupting current sequence.')
            # Cancel current navigation if any
            if self.current_goal_handle:
                try:
                    self.current_goal_handle.cancel_goal_async()
                    self.get_logger().info('Cancelled current navigation goal')
                except:
                    pass
        
        # Split by comma and clean up each command
        command_string = msg.data.strip()
        self.action_queue = [cmd.strip() for cmd in command_string.split(',') if cmd.strip()]
        self.current_action_index = 0
        self.sequence_active = True
        self.sequence_id += 1
        self.retry_count = 0  # Reset retry count for new sequence
        
        self.get_logger().info(f'Starting sequence {self.sequence_id} with {len(self.action_queue)} actions')
        self.get_logger().info(f'Commands: {self.action_queue}')
        
        # Publish sequence start status
        status_msg = f'SEQUENCE:{self.sequence_id}:STARTED:{len(self.action_queue)} actions'
        self.publish_sequence_status(status_msg)
        
        # Start the first action
        self.execute_next_action()
    
    def execute_next_action(self):
        """Execute the next action in the sequence"""
        if not self.sequence_active or self.current_action_index >= len(self.action_queue):
            self.complete_sequence()
            return
        
        current_action = self.action_queue[self.current_action_index]
        
        # Publish sequence status
        status_msg = f'SEQUENCE:{self.sequence_id}:EXECUTING:{self.current_action_index + 1}/{len(self.action_queue)}:"{current_action}"'
        self.publish_sequence_status(status_msg)
        
        self.get_logger().info(f'Executing action {self.current_action_index + 1}/{len(self.action_queue)}: "{current_action}"')
        
        # Process the action using existing command processing logic
        self.process_command_for_sequence(current_action)
    
    def process_command_for_sequence(self, command):
        """Process a single command in sequence context"""
        command = command.strip()
        
        # Check if it's a navigation command
        nav_commands = self.command_map.get('navigation_commands', {})
        if command in nav_commands:
            target_name = nav_commands[command]
            self.get_logger().info(f'Processing sequence navigation command: {command} -> {target_name}')
            
            # Use existing navigation logic
            if target_name not in self.objects:
                error_msg = f'SEQUENCE:FAILED:name={target_name} - object not found'
                self.get_logger().error(f'Object "{target_name}" not found in map')
                self.publish_result(error_msg)
                return
                
            # Get object coordinates and navigate
            obj_data = self.objects[target_name]
            try:
                x = float(obj_data['x'])
                y = float(obj_data['y'])
                yaw = float(obj_data['yaw'])
                frame_id = obj_data.get('frame_id', 'map')
                
                pose = create_pose_stamped(x, y, yaw, frame_id)
                tag = f"sequence={self.sequence_id}:action={self.current_action_index + 1}:command={command}"
                
                self.go_to(pose, tag)
                
            except (KeyError, ValueError, TypeError) as e:
                error_msg = f'SEQUENCE:FAILED:command={command} - invalid object data: {e}'
                self.get_logger().error(error_msg)
                self.publish_result(error_msg)
            return
        
        # Check if it's a special command
        special_commands = self.command_map.get('special_commands', {})
        if command in special_commands:
            action = special_commands[command]
            self.get_logger().info(f'Processing sequence special command: {command} -> {action}')
            result_msg = f'SEQUENCE:SPECIAL:{action}:command={command}'
            self.publish_result(result_msg)
            return
            
        # Unknown or unsupported command - skip it
        self.get_logger().warn(f'Skipping unsupported sequence command: "{command}"')
        result_msg = f'SEQUENCE:SKIPPED:command={command}'
        self.publish_result(result_msg)
    
    def handle_sequence_progress(self, result_str):
        """Handle progress in sequence execution"""
        # Check if current action succeeded, failed, or was skipped
        if 'SUCCEEDED' in result_str or 'SKIPPED' in result_str or 'SPECIAL' in result_str:
            # Move to next action
            self.current_action_index += 1
            self.execute_next_action()
        elif 'FAILED' in result_str:
            # For now, continue with next action even if one fails
            self.get_logger().warn(f'Action failed, continuing with next action: {result_str}')
            self.current_action_index += 1
            self.execute_next_action()
    
    def complete_sequence(self):
        """Complete the current sequence"""
        self.get_logger().info(f'Sequence {self.sequence_id} completed')
        
        status_msg = f'SEQUENCE:{self.sequence_id}:COMPLETED:ALL:{len(self.action_queue)} actions processed'
        self.publish_sequence_status(status_msg)
        
        # Reset sequence state
        self.sequence_active = False
        self.action_queue = []
        self.current_action_index = 0
    
    def publish_sequence_status(self, status_str):
        """Publish sequence status"""
        msg = String()
        msg.data = status_str
        self.sequence_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = TargetNavNode()
    
    if not node.objects:
        node.get_logger().error('Failed to initialize - no objects loaded')
        rclpy.shutdown()
        return
    
    # Use MultiThreadedExecutor to handle concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info('Target navigation node spinning...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Target navigation node interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()