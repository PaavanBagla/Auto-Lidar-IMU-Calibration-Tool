# Auto-Calibration-Tool

# Automatic Calibration Tool Usage Instructions
Usage: """  ./run_lidar2imu <lidar_pcds_dir> <lidar_pose_file> <extrinsic_json>  """

(base) paavan@paavan-ubuntu20:~$ ~/SensorsCalibration/lidar2imu/auto_calib/bin/run_lidar2imu ~/SensorsCalibration/lidar2imu/auto_calib/data/top_center_lidar/ ~/SensorsCalibration/lidar2imu/auto_calib/data/NovAtel-pose-lidar-time.txt ~/SensorsCalibration/lidar2imu/auto_calib/data/gnss-to-top_center_lidar-extrinsic.json
gnss-to-top_center_lidar-extrinsic

--------------------------------------------------------
# Data Requirements for Running the Calibration Tool

1. Lidar PCDs Directory:
- Description: A directory containing Lidar point cloud data files in .pcd format.
- Preparation: The Lidar data should be collected and saved in .pcd format. Organize these files in a directory that you can specify as an input to the tool.

2. Lidar Pose File: // IMU Data
- Description: A text file containing pose information for the Lidar data, similar to the NovAtel-pose-lidar-time.txt file in the sample data.
- Preparation: Collect pose data for your Lidar sensor, ensuring it is synchronized with the Lidar data. Format this data in a text file with timestamps and pose information.
Sample Data:
2021-10-26-16-21-29-468 1.000000000 -0.000014703 -0.000006041 0.000061155 0.000014701 1.000000000 0.000004847 0.000095810 0.000006040 -0.000004848 1.000000000 -0.000077579
- First part of each line (2021-10-26-16-21-29-468) is a timestamp indicating the date and time when the data was recorded. 
- The next nine numbers form a 3x3 rotation matrix which represents the orientation of the vehicle in 3D space.
- The last three numbers represent a translation vector that indicates the position of the vehicle in the global coordinate system.

3. Extrinsic JSON File:
Description: A JSON file containing the initial extrinsic calibration parameters between the Lidar and another reference frame (e.g., GNSS or vehicle frame).
Preparation: Perform an initial calibration to determine the transformation matrix between your Lidar sensor and the reference frame. Format this matrix in a JSON file similar to the sample data.

----------------------------------------------------------
# Organize Data:
Organize your data files in a directory structure similar to the sample data to ensure compatibility with the tool's expected input paths.


# Running the Tool on Your Data:
~/SensorsCalibration/lidar2imu/auto_calib/bin/run_lidar2imu <path_to_your_lidar_data> <path_to_your_imu_data> <path_to_your_extrinsic_file>


