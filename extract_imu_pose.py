#!/usr/bin/env python3

"""
This script takes a rosbag and an output file as an input.
It subscribes to the the imu topic from the rosbag and extracts the orientation vector which is the quaternion (x, y, z, w).
Converts it into a Rotation Matrix and Initializes a Translation Vector.
Output a txt file formatted accordingly with the auto_calib tool requirements i.e. Timestamp 9-values(3x3_rotation_matrix) 3-values(translation_vector)
"""
'''
import rosbag
import tf.transformations as tf
from datetime import datetime
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys

def quaternion_to_rotation_matrix(x, y, z, w):
    """Converts Quaternion to a 3x3 rotation matrix"""
    # # Normalize the quaternion
    # norm = (x**2 + y**2 + z**2 + w**2)**0.5
    # x, y, z, w = x / norm, y / norm, z / norm, w / norm

    # # Calculate the rotation matrix elements
    # R = [
    #     [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
    #     [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
    #     [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    # ]
    # return R
    return tf.quaternion_matrix([x, y, z, w])[:3, :3] # Extracting the upper 3x3 matrix from the 4x4 matrix given by the tf function

def main(bag_path, output_file):
    with rosbag.Bag(bag_path, 'r') as bag, open(output_file, 'w') as file:
        for topic, msg, t in bag.read_messages(topics=['/imu/data_raw', '/novatel/oem7/odom']): #/gps/imu
            # Extracting timestamp
            timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9 # The script combines secs and nsecs to create a full timestamp
            dt_object = datetime.fromtimestamp(timestamp) # Convert a Unix timestamp into a datetime object in Python.
            formatted_time = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]

            if topic == '/imu/data_raw': # '/gps/imu'
                # Converting quaternion to rotation matrix
                rotation_matrix = quaternion_to_rotation_matrix(
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                )
                # Assuming zero translation if odometry data is not available
                translation_vector = [0.0, 0.0, 0.0]
                # translation_vector = [-0.05, 0.61, 1.54] # x = 0.05 left, y = .61 forward, z = 1.54 up

            elif topic == '/novatel/oem7/odom':
                # Extracting position for translation vector
                translation_vector = [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]

            # Write to file in the desired format
            file.write(f"{formatted_time} ")
            for row in rotation_matrix:
                file.write(" ".join(f"{val:.9f}" for val in row) + " ")
            file.write(" ".join(f"{val:.9f}" for val in translation_vector) + "\n")

    print(f"IMU data extracted and saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: extract_imu_pose.py <bag_path> <output_file>")
        sys.exit(1)

    bag_path = sys.argv[1]
    output_file = sys.argv[2]
    main(bag_path, output_file)

'''


"""
This script takes a rosbag and an output file as an input.
It subscribes to the odometry topic from the rosbag and extracts the pose information.
Converts the quaternion into a Rotation Matrix and uses the position as the Translation Vector.
Outputs a txt file formatted accordingly with the auto_calib tool requirements i.e. Timestamp 9-values(3x3_rotation_matrix) 3-values(translation_vector)
"""

import rosbag
import tf.transformations as tf
from datetime import datetime
from nav_msgs.msg import Odometry
import sys

def quaternion_to_rotation_matrix(x, y, z, w):
    """Converts Quaternion to a 3x3 rotation matrix"""
    return tf.quaternion_matrix([x, y, z, w])[:3, :3]  # Extracting the upper 3x3 matrix from the 4x4 matrix given by the tf function

def main(bag_path, output_file):
    with rosbag.Bag(bag_path, 'r') as bag, open(output_file, 'w') as file:
        for topic, msg, t in bag.read_messages(topics=['/novatel/oem7/odom']):
            # Extracting timestamp
            timestamp = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
            dt_object = datetime.fromtimestamp(timestamp)
            formatted_time = dt_object.strftime('%Y-%m-%d-%H-%M-%S-%f')[:-3]

            # Converting quaternion to rotation matrix
            rotation_matrix = quaternion_to_rotation_matrix(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )

            # Extracting position for translation vector
            translation_vector = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]

            # Write to file in the desired format
            file.write(f"{formatted_time} ")
            for row in rotation_matrix:
                file.write(" ".join(f"{val:.9f}" for val in row) + " ")
            file.write(" ".join(f"{val:.9f}" for val in translation_vector) + "\n")

    print(f"Odometry data extracted and saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: extract_odom_pose.py <bag_path> <output_file>")
        sys.exit(1)

    bag_path = sys.argv[1]
    output_file = sys.argv[2]
    main(bag_path, output_file)