from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

# create reader instance and open for reading
with Reader('pool_map/') as reader:
    msg = None
    # iterate over messages
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/slam_out_pose':
            pose_msg = deserialize_cdr(rawdata, connection.msgtype)
            pose_info = {'px': pose_msg.pose.position.x, 'py': pose_msg.pose.position.y}
            print(pose_info)
