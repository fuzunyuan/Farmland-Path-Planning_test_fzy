import rclpy
import yaml
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def load_path_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    return data

def publish_path_once():
    rclpy.init()
    node = rclpy.create_node('path_publisher_once')
    # 创建发布器
    publisher = node.create_publisher(Path, '/path', 10)
    # 读取路径点
    path_data = load_path_from_yaml('./a.yaml')
    # 创建 Path 消息
    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = node.get_clock().now().to_msg()
    for x, y in path_data:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 默认朝向
        path_msg.poses.append(pose)
    # 发布消息一次
    publisher.publish(path_msg)
    node.get_logger().info('路径已发布到 /path 话题')
    # 稍作延时以确保消息送出
    time.sleep(0.5)
    # 清理资源退出
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    publish_path_once()

