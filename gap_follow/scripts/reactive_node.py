#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.last_processed_ranges = np.zeros((1080,),dtype = np.float32) # create zeros array as a free space

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        max_value = 1.5
        proc_ranges = np.array(ranges, dtype=np.float32) # store ranges as np array
        ranges = (proc_ranges + self.last_processed_ranges) / 2 # average with previous scan
        self.last_processed_ranges = proc_ranges # update previous scan
        np.clip(ranges, 0, max_value, out=ranges) # clipping high ranges value
        return ranges
    
    def find_closest_point(self, ranges):
        """Find closest point to LiDAR
        """
        closest_index = min(range(len(ranges)), key=lambda x: ranges[x])
        return closest_index
    
    def preprocess_bubble(self, ranges, closest_point):
        """Eliminate all points inside 'bubble' (set them to zero) 
        """
        left = max(0, closest_point - 100)
        right = min(len(ranges) - 1, closest_point + 99)
        for i in range(left, right + 1):
            ranges[i] = 0
        return ranges

    def find_max_gap(self, ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        split_idx = np.where(ranges == 0.0)[0]
        sranges = np.split(ranges,split_idx)
        len_sranges = np.array([len(x) for x in sranges])
        max_idx = np.argmax(len_sranges)
        if max_idx == 0:
            start_i = 0
            end_i = len_sranges[0]-1
        else:
            start_i = np.sum(len_sranges[:max_idx])
            end_i = start_i+len_sranges[max_idx]-1
        max_length_ranges = sranges[max_idx]
        return start_i, end_i, max_length_ranges
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        gap_length = end_i - start_i + 1
        best_idx = start_i + round(gap_length / 2)
        return best_idx

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        closest_point = self.find_closest_point(proc_ranges)

        #Eliminate all points inside 'bubble' (set them to zero) 
        proc_bubble = self.preprocess_bubble(proc_ranges, closest_point)

        #Find max length gap 
        max_gap_start, max_gap_end, max_gap_ranges = self.find_max_gap(proc_bubble)

        #Find the best point in the gap 
        best_point = self.find_best_point(max_gap_start, max_gap_end, max_gap_ranges)

        #Publish Drive message
        steering_angle = (best_point * data.angle_increment) + data.angle_min

        if 0 < steering_angle < np.pi/18: # 0 degree < angle < 10 degree
            velocity = 2.0
        elif np.pi/18 <= steering_angle < np.pi/9: # 10 degree <= angle < 20 degree
            velocity = 1.0
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle
        self.publisher_drive.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()