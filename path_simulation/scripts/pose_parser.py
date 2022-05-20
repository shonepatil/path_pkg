from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd


rosbag_name_and_path = "/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_nav2_pkg/ros_data/rosbags/pool_donut_map/"
output_csv_name_and_path = "/home/projects/ros2_ws/src/ucsd_robocar_hub2/ucsd_robocar_path2_pkg/paths/pool_donut_poses.csv"
# create reader instance and open for reading

with Reader(rosbag_name_and_path) as reader:
    msg = None
    # iterate over messages
    df = pd.DataFrame(columns = ['px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw'])
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/slam_out_pose':
            pose_msg = deserialize_cdr(rawdata, connection.msgtype)
            df = df.append({'px': pose_msg.pose.position.x, 'py': pose_msg.pose.position.y}, ignore_index=True)
    df.to_csv(output_csv_name_and_path, index = False)
