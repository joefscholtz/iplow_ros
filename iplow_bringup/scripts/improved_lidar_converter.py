#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import array
import copy
import time

class ImprovedLivoxConverter(Node):
    def __init__(self):
        super().__init__('improved_livox_converter')
        
        # Create a subscription to the Livox LiDAR topic
        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.lidar_callback,
            10)
        
        # Create a publisher for the converted messages
        self.publisher = self.create_publisher(
            PointCloud2,
            '/converted/lidar',
            10)
        
        # Store timing information
        self.last_timestamp = None
        self.sequence = 0
        
        # Get parameter for debugging
        self.debug_mode = self.declare_parameter('debug', False).value
        
        self.get_logger().info('Improved Livox converter node started')

    def lidar_callback(self, msg):
        try:
            # Create new PointCloud2 message
            new_msg = PointCloud2()
            new_msg.header = copy.deepcopy(msg.header)
            
            # Use real ROS timestamp
            current_time = self.get_clock().now()
            new_msg.header.stamp = current_time.to_msg()
            
            # Calculate realistic time difference (around 0.1s is normal for 10Hz)
            time_diff = 0.1
            if self.last_timestamp is not None:
                time_diff = (current_time.nanoseconds - self.last_timestamp) / 1e9
                if self.debug_mode and time_diff > 0.2:
                    self.get_logger().info(f'Time gap: {time_diff:.3f}s')
            
            self.last_timestamp = current_time.nanoseconds
            
            # Find field positions
            timestamp_offset = None
            point_step = msg.point_step
            
            for field in msg.fields:
                if field.name == 'timestamp':
                    timestamp_offset = field.offset
                    break
            
            if timestamp_offset is None:
                self.get_logger().warn('No timestamp field found in PointCloud2 message')
                # Just republish the message as is with updated header timestamp
                self.publisher.publish(new_msg)
                return
            
            # Copy all fields
            new_msg.height = msg.height
            new_msg.width = msg.width
            new_msg.fields = copy.deepcopy(msg.fields)
            new_msg.is_bigendian = msg.is_bigendian
            new_msg.point_step = msg.point_step
            new_msg.row_step = msg.row_step
            new_msg.is_dense = msg.is_dense
            
            # Create a copy of the data
            data_bytes = bytearray(msg.data)
            
            # Now modify timestamp in each point
            # Replace the timestamp with a realistic value (in nanoseconds)
            seq_ns = float(self.sequence) * 1e8  # Small incremental value to make each point unique
            base_ns = current_time.nanoseconds
            
            for i in range(0, len(data_bytes), point_step):
                if i + timestamp_offset + 8 <= len(data_bytes):  # Make sure we don't go out of bounds
                    # Create an increasing timestamp for each point
                    point_time = base_ns + seq_ns
                    
                    # Pack the new timestamp (8 bytes) into the data array
                    # Use little endian (standard) for packing
                    timestamp_bytes = struct.pack('<d', point_time)
                    
                    # Replace the bytes in the data array
                    for j in range(8):
                        if i + timestamp_offset + j < len(data_bytes):
                            data_bytes[i + timestamp_offset + j] = timestamp_bytes[j]
            
            new_msg.data = bytes(data_bytes)
            self.sequence += 1
            
            # Publish the modified message
            self.publisher.publish(new_msg)
            
            if self.debug_mode and self.sequence % 50 == 0:
                self.get_logger().info(f'Processed frame {self.sequence}: {new_msg.width} points')
                
        except Exception as e:
            self.get_logger().error(f'Error processing PointCloud2: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    converter = ImprovedLivoxConverter()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        converter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
