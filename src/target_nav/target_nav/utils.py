import math
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler

def normalize_yaw(yaw):
    """Normalize yaw angle to [-π, π] range"""
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw

def yaw_to_quaternion(yaw):
    """Convert yaw angle to quaternion (roll=0, pitch=0)"""
    normalized_yaw = normalize_yaw(yaw)
    q = quaternion_from_euler(0.0, 0.0, normalized_yaw)
    quaternion = Quaternion()
    quaternion.x = q[0]
    quaternion.y = q[1]
    quaternion.z = q[2]
    quaternion.w = q[3]
    return quaternion

def create_pose_stamped(x, y, yaw, frame_id="map"):
    """Create PoseStamped message from x, y, yaw coordinates"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quaternion(yaw)
    return pose

def extract_pose_info(pose_stamped):
    """Extract (x, y, yaw) from PoseStamped message for logging"""
    x = pose_stamped.pose.position.x
    y = pose_stamped.pose.position.y
    q = pose_stamped.pose.orientation
    
    # Convert quaternion to yaw (assuming roll=pitch=0)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return x, y, yaw