import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np
import math
import time

#Topics & Subs, Pubs
LIDAR_TOPIC_NAME = '/scan'
ERROR_TOPIC_NAME = '/error'
NODE_NAME = 'tube_follower_node'

class TubeFollower(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.camera_subscriber = self.create_subscription(LaserScan, LIDAR_TOPIC_NAME, self.lidar_callback, 10)
        self.camera_subscriber
        self.error_publisher = self.create_publisher(Float32MultiArray, ERROR_TOPIC_NAME, 10)
        self.error_publisher
        self.error_msg = Float32MultiArray() # [error_distance_to_wall, heading_error_with_wall]
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim.append(MultiArrayDimension())
        self.error_msg.layout.dim[0].label = "lateral_error"
        self.error_msg.layout.dim[1].label = "longitdudinal_error"
        self.error_msg.layout.dim[2].label = "heading_error"
        self.error_msg.layout.dim[0].size = 1
        self.error_msg.layout.dim[1].size = 1
        self.error_msg.layout.dim[2].size = 1

        # Lidar info
        self.default_desired_lateral_distance = 1
        self.default_desired_longitudinal_distance = 1
        self.default_viewing_angle = 360
        self.default_degree_window_size = 0
        self.default_front_degree_angle = 0
        self.default_right_degree_angle = 90
        self.default_left_degree_angle = 270

        self.num_scans = 0
        self.scans_per_degree = 0
        self.lateral_degree_window = 0
        self.lateral_start_angle = 0
        self.lateral_end_angle = 0
        self.longitudinal_degree_window = 0
        self.longitudinal_start_angle = 0
        self.longitudinal_end_angle = 0

        # wall following parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('desired_lateral_distance', self.default_desired_lateral_distance),
                ('desired_longitudinal_distance', self.default_desired_longitudinal_distance),
                ('viewing_angle', self.default_viewing_angle),
                ('degree_window_size', self.default_degree_window_size),
                ('front_degree_angle', self.default_front_degree_angle),
                ('right_degree_angle', self.default_right_degree_angle),
                ('left_degree_angle', self.default_left_degree_angle)
            ])

        self.reff_lat_wall_dist = self.get_parameter('desired_lateral_distance').value
        self.reff_lon_wall_dist = self.get_parameter('desired_longitudinal_distance').value
        self.viewing_angle = self.get_parameter('viewing_angle').value
        self.degree_window_size = self.get_parameter('degree_window_size').value
        self.front_degree_angle = self.get_parameter('front_degree_angle').value
        self.right_degree_angle = self.get_parameter('right_degree_angle').value
        self.left_degree_angle = self.get_parameter('left_degree_angle').value

    def lidar_callback(self, data):
        scan_ranges = data.ranges
        self.num_scans = len(scan_ranges)
        self.scans_per_degree = float(self.num_scans/self.viewing_angle)
        # self.get_logger().info(f'num_scans,scans_per_degree,viewing_angle:{self.num_scans},{self.scans_per_degree},{self.viewing_angle}')
        theta_w = self.degree_window_size / 2
        
        # right
        lateral_right_start_angle = self.right_degree_angle - theta_w
        lateral_right_end_angle = self.right_degree_angle + theta_w
        # left
        lateral_left_start_angle = self.left_degree_angle - theta_w
        lateral_left_end_angle = self.left_degree_angle + theta_w
        # front
        longitudinal_start_angle = self.front_degree_angle - theta_w
        longitudinal_end_angle = self.front_degree_angle + theta_w

        # Viewing windows
        lateral_right_degree_window = np.array(scan_ranges[int(lateral_right_start_angle*self.scans_per_degree):int(lateral_right_end_angle*self.scans_per_degree)])
        lateral_left_degree_window = np.array(scan_ranges[int(lateral_left_start_angle*self.scans_per_degree):int(lateral_left_end_angle*self.scans_per_degree)])
        long_start = np.array(scan_ranges[int(longitudinal_start_angle*self.scans_per_degree):])
        long_end = np.array(scan_ranges[:int(longitudinal_end_angle*self.scans_per_degree)])
        longitudinal_degree_window = np.concatenate([long_start, long_end])

        # remove nan/inf
        lateral_right_degree_window = lateral_right_degree_window[~np.isnan(lateral_right_degree_window)]
        lateral_right_degree_window = lateral_right_degree_window[~np.isinf(lateral_right_degree_window)]
        lateral_left_degree_window = lateral_left_degree_window[~np.isnan(lateral_left_degree_window)]
        lateral_left_degree_window = lateral_left_degree_window[~np.isinf(lateral_left_degree_window)]
        longitudinal_degree_window = longitudinal_degree_window[~np.isnan(longitudinal_degree_window)]
        longitudinal_degree_window = longitudinal_degree_window[~np.isinf(longitudinal_degree_window)]

        try:
            # right path angle distances
            x1 = float(lateral_right_degree_window[0]) * math.cos(math.radians(theta_w))
            x2 = float(lateral_right_degree_window[-1]) * math.cos(math.radians(theta_w))
            y1 = float(lateral_right_degree_window[0]) * math.sin(math.radians(theta_w))
            y2 = float(lateral_right_degree_window[-1]) * math.sin(math.radians(theta_w))

            delta_x_right = x2 - x1 
            delta_y_right = y1 + y2
            
            # left path angle distances
            x3 = float(lateral_left_degree_window[0]) * math.cos(math.radians(theta_w))
            x4 = float(lateral_left_degree_window[-1]) * math.cos(math.radians(theta_w))
            y3 = float(lateral_left_degree_window[0]) * math.sin(math.radians(theta_w))
            y4 = float(lateral_left_degree_window[-1]) * math.sin(math.radians(theta_w))
            
            delta_x_left = x4 - x3
            delta_y_left = y4 + y3
            
            # Distance to all walls
            right_average_side_distance = np.mean(lateral_right_degree_window)
            left_average_side_distance = np.mean(lateral_left_degree_window)
            front_average_distance = np.mean(longitudinal_degree_window)
            lateral_difference = left_average_side_distance - right_average_side_distance

            right_path_heading = abs(float(np.arctan2(delta_x_right, delta_y_right)))
            left_path_heading = abs(float(np.arctan2(delta_x_left, delta_y_left)))

            # errors for PID
            lateral_error = self.reff_lat_wall_dist - lateral_difference
            longitdudinal_error = -1 * min(0, (front_average_distance - self.reff_lon_wall_dist))
            heading_error = (-1 * np.sign(delta_x_right) * min((math.pi/3),(np.mean([right_path_heading, left_path_heading])))) / (math.pi/3)

            # self.get_logger().info(f'\n'
            #                     f'\n right start angle:{lateral_right_start_angle}'
            #                     f'\n right end angle:{lateral_right_end_angle}'
            #                     f'\n left start angle:{lateral_left_start_angle}'
            #                     f'\n left end angle:{lateral_left_end_angle}'
            #                     f'\n left start distance:{lateral_left_degree_window[0]}'
            #                     f'\n left end distance:{lateral_left_degree_window[-1]}'
            #                     f'\n right start distance:{lateral_right_degree_window[0]}'
            #                     f'\n right end distance:{lateral_right_degree_window[-1]}'
            #                     f'\n '
            #                     f'\n '
            #                     f'\n delta_x_right:{delta_x_right}'
            #                     f'\n delta_y_right:{delta_y_right}'
            #                     f'\n right_path_heading:{right_path_heading}'
            #                     f'\n '
            #                     f'\n '
            #                     f'\n delta_x_left:{delta_x_left}'
            #                     f'\n delta_y_left:{delta_y_left}'
            #                     f'\n left_path_heading:{left_path_heading}'
            #                     f'\n '
            #                     f'\n '
            #                     f'\n heading_error:{heading_error}'
            #                         )

            # Publish
            self.error_msg.data = [float(lateral_error), float(longitdudinal_error), float(heading_error)]
            self.error_publisher.publish(self.error_msg)
            # self.get_logger().info(f'\n'
            #                     f'\n lateral_right_degree_window:{lateral_right_degree_window}'
            #                     f'\n lateral_left_degree_window:{lateral_left_degree_window}'
            #                     f'\n longitudinal_degree_window:{longitudinal_degree_window}'
            #                         )
            # self.get_logger().info(f'\n'
            #                     f'\n right_average_side_distance:{right_average_side_distance}'
            #                     f'\n left_average_side_distance:{left_average_side_distance}'
            #                     f'\n front_average_distance:{front_average_distance}'
            #                     f'\n lateral_difference:{lateral_difference}'
            #                         )
        except IndexError:
            pass


def main(args=None):
    rclpy.init(args=args)
    tube_follower_publisher = TubeFollower()
    try:
        rclpy.spin(tube_follower_publisher)
        tube_follower_publisher.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        tube_follower_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')
        time.sleep(1)
        tube_follower_publisher.destroy_node()
        rclpy.shutdown()
        tube_follower_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()
