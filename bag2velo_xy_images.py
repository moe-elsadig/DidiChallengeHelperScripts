import sensor_msgs.point_cloud2 as pc2
import argparse
import rosbag
import numpy as np
import os
import matplotlib.pyplot as plt
from mayavi import mlab

cwd = os.getcwd()
rosbag_file = None
m_dpi = 1400

def extract_pc_images():
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

    directory = cwd + "/" + rosbag_file + "_velo_xy_images_c"
    print "Converting to images...\n"

    if not os.path.exists(directory):
        os.makedirs(directory)

    count = 0
    for cloud in pc_clouds:
        x_s = []
        y_s = []
        z_s = []
        i_s = []
        r_s = []
        for x,y,z,i,r in cloud:
            x_s.append(x)
            y_s.append(y)
            z_s.append(z)
            i_s.append(i)
            r_s.append(r)


        fig = plt.figure(frameon = False)
        cm = plt.cm.get_cmap('Spectral')
        colour = np.add(np.abs(x_s),np.abs(y_s))
        sc = plt.scatter(x_s, y_s, c=colour, vmin=0, vmax=150, s=0.05, edgecolors= '', cmap=cm)
        plt.axis('tight')
        ax = plt.gca()
        ax.axes.get_xaxis().set_visible(False)
        ax.axes.get_yaxis().set_visible(False)
        ax.patch.set_facecolor('black')

        plt.savefig((directory+"/"+rosbag_file+"_velo_xy_images_c_"+str(count)+".png"), bbox_inches='tight', dpi=m_dpi)
        count += 1

    # Save the results in .NPY format
    print "Conversion Done."
    print "Done.\nFile saved in dir: ", directory, "/...\n"


parser = argparse.ArgumentParser(description='ROSBAG File: Images from Pointcloud2 from ROSBAG')
parser.add_argument(
	'bag',
	type=str,
	help='Path to rosbag file'
)
parser.add_argument(
	'dpi',
	type=int,
	help='Path to rosbag file'
)
args = parser.parse_args()
rosbag_file = args.bag
m_dpi = args.dpi

extract_pc_images()
