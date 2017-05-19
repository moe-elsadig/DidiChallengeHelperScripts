# sequence:
# 1- roscore
# 2-rosrun velodyne_pointcloud cloud_node _calibration:=/opt/ros/kinetic/share/velodyne_pointcloud/params/32db.yaml
# 3- rosrun rosbag record -O point_cloud_only.bag /velodyne_points
# 4- rosbag play filename.bag




import sensor_msgs.point_cloud2 as pc2
import argparse
import rosbag
import numpy as np
import os

cwd = os.getcwd()
rosbag_file = None

def extract_pc_points():
    #select the rosbag file you wish to check
    bag = rosbag.Bag(rosbag_file, 'r')
    bag_img_msg = bag.read_messages(topics=["/velodyne_points"])

    pc_clouds = []
    points_per_cloud = []
    # Obtain one lidar point cloud frame
    print "Extracting..."
    for topic, msg, time in bag_img_msg:
        cloud_gen = pc2.read_points(msg)
        cloud = []
        point_count = 0
        for x,y,z,intensity,ring in cloud_gen:
            cloud.append([x,y,z,intensity,ring])
            point_count += 1
        pc_clouds.append(cloud)
        points_per_cloud.append(point_count)

    # Display results
    print "Done.\nThe number of clouds extracted is: ", len(pc_clouds)
    print "The Average number of points per frame is: ", np.average(points_per_cloud)
    print "The max and min number of points per frame is: ", np.max(points_per_cloud),"\t", np.min(points_per_cloud)

    # Save the results in .NPY format
    print "Saving extraction..."
    directory = cwd + "/" + rosbag_file + "_" + "format_XYZIR"
    np.save(directory, pc_clouds)
    print "Done.\nFile saved in dir/file", directory


parser = argparse.ArgumentParser(description='ROSBAG File: Pointcloud2 from ROSBAG format_XYZIR')
parser.add_argument(
	'bag',
	type=str,
	help='Path to rosbag file'
)

args = parser.parse_args()
rosbag_file = args.bag

extract_pc_points()
