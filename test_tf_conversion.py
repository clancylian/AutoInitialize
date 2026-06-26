#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

class TfConversionTest(Node):
    def __init__(self):
        super().__init__('tf_conversion_test')
        
        self.subscription = self.create_subscription(
            Odometry,
            '/rkbot/lio/odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('TF conversion test node ready')

    def odom_callback(self, msg):
        try:
            # 获取 world 到 map 的变换
            trans = self.tf_buffer.lookup_transform(
                'rkbot/map',
                'rkbot/world',
                rclpy.time.Time())
            
            self.get_logger().info('\n=== TF Conversion Test ===')
            self.get_logger().info(f'World->Map Transform:')
            self.get_logger().info(f'  Translation: [{trans.transform.translation.x:.3f}, '
                                  f'{trans.transform.translation.y:.3f}, '
                                  f'{trans.transform.translation.z:.3f}]')
            
            # 构建变换矩阵
            # T_map_world = [R  t]
            #               [0  1]
            trans_x = trans.transform.translation.x
            trans_y = trans.transform.translation.y
            trans_z = trans.transform.translation.z
            
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            
            # 将四元数转换为旋转矩阵
            rot = R.from_quat([qx, qy, qz, qw])
            rot_mat = rot.as_matrix()
            
            # 4x4 变换矩阵
            T_map_world = np.eye(4)
            T_map_world[:3, :3] = rot_mat
            T_map_world[:3, 3] = [trans_x, trans_y, trans_z]
            
            # 获取 odom 在 world 坐标系下的位姿
            world_x = msg.pose.pose.position.x
            world_y = msg.pose.pose.position.y
            world_z = msg.pose.pose.position.z
            
            world_qx = msg.pose.pose.orientation.x
            world_qy = msg.pose.pose.orientation.y
            world_qz = msg.pose.pose.orientation.z
            world_qw = msg.pose.pose.orientation.w
            
            # 将 odom 位姿转换为齐次坐标
            odom_world_homogeneous = np.array([world_x, world_y, world_z, 1.0])
            
            # 应用变换：T_map_odom = T_map_world * T_world_odom
            odom_map_homogeneous = T_map_world @ odom_world_homogeneous
            
            # 计算旋转：R_map_odom = R_map_world * R_world_odom
            odom_rot_world = R.from_quat([world_qx, world_qy, world_qz, world_qw])
            odom_rot_map = rot * odom_rot_world
            odom_q_map = odom_rot_map.as_quat()
            
            self.get_logger().info(f'\nOdometry in World frame:')
            self.get_logger().info(f'  Position: [{world_x:.3f}, {world_y:.3f}, {world_z:.3f}]')
            self.get_logger().info(f'  Orientation: [{world_qx:.3f}, {world_qy:.3f}, {world_qz:.3f}, {world_qw:.3f}]')
            
            self.get_logger().info(f'\nOdometry in Map frame:')
            self.get_logger().info(f'  Position: [{odom_map_homogeneous[0]:.3f}, '
                                  f'{odom_map_homogeneous[1]:.3f}, '
                                  f'{odom_map_homogeneous[2]:.3f}]')
            self.get_logger().info(f'  Orientation: [{odom_q_map[0]:.3f}, '
                                  f'{odom_q_map[1]:.3f}, '
                                  f'{odom_q_map[2]:.3f}, '
                                  f'{odom_q_map[3]:.3f}]')
            self.get_logger().info('=========================\n')
            
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'TF connectivity error: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TfConversionTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
