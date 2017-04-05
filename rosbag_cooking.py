import rosbag
import yaml
import numpy as np
from PIL import Image
import cv2
import os
# import pandas as pd
import argparse

parser = argparse.ArgumentParser(description='ROSBAG File: Cooking ROSBAGs')
parser.add_argument(
	'bag',
	type=str,
	help='Path to rosbag file'
)

args = parser.parse_args()
rosbag_file = args.bag

#select the rosbag file you wish to check
bag = rosbag.Bag(rosbag_file, 'r')

#Get summary information about the bag
info_dict = yaml.load(bag._get_yaml_info())

#print all the key value pairs in the info summary
for k, val in info_dict.iteritems():
    print "key: ", k, "\nvalue: ", val
print "\n"

#function to get the type and topics information from the bag
topics = bag.get_type_and_topic_info()[1].keys()

#Iterate over the topics and return a list of the types
types = []
for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
    types.append(bag.get_type_and_topic_info()[1].values()[i][0])

print "\nThe topics in this rosbag file are: \n"
for i in range(len(types)):
    print i, ": ", types[i]
print "\n\n"




# TASK 1
print "\n\nChecking for Images now...\n\n"

#get the messages of the entered topic
bag_img_msg = bag.read_messages(topics=["/image_raw"])
print "The topic chosen: /image_raw \n"

#check the type of the returned messages
print "The type of the returned message is: ", type(bag_img_msg), "\n"

#print a sample from the generator
print "\nChecking the a sample from the generator...\n"
topic, msg, time_stamp = bag_img_msg.next()
print "The type of output from the generator is: ", type(bag_img_msg.next()), "\n"
print "\nInside this item are the values: \n"
print type(topic), "\n",type(msg), "\n",type(time_stamp), "\n"
print "The string value: ", topic, "\n"

print "The attributes and methods associated with the second value: ", dir(msg), "\n"
print "The attributes and methods associated with the third value: ", dir(time_stamp), "\n"

print "The return type for the function:\tdata\tis:\t", type(msg.data), "\n"
print "The return type for the function:\tdeserialize\tis:\t", type(msg.deserialize), "\n"
print "The return type for the function:\tdeserialize_numpy\tis:\t", type(msg.deserialize_numpy), "\n"
print "The return type for the function:\tencoding\tis:\t", type(msg.encoding), "\n"
print "The return type for the function:\theader\tis:\t", type(msg.header), "\n"
print "The return type for the function:\theight\tis:\t", type(msg.height), "\n"
print "The return type for the function:\tis_bigendian\tis:\t", type(msg.is_bigendian), "\n"
print "The return type for the function:\tserialize\tis:\t", type(msg.serialize), "\n"
print "The return type for the function:\tserialize_numpy\tis:\t", type(msg.serialize_numpy), "\n"
print "The return type for the function:\tstep\tis:\t", type(msg.step), "\n"
print "The return type for the function:\twidth\tis:\t", type(msg.width), "\n"

img_w = msg.width
img_h = msg.height
print "Testing methods for value 2:\nwidth\n", img_w, "\nheight\n", img_h,"\n"

print "The values data type is: ", type(msg.data), "\n"
print msg.data[:20], "...\nunknown string needs to be coverted to int value using numpy\n"

val_img_vec = np.fromstring(msg.data, dtype=np.uint8)
print "The new value type is: ", type(val_img_vec), "\nand the shape is: ", val_img_vec.shape, "\n"

print "Reshape the vector back to the given height and width using PIL and convert to IMG:\n"

val_img_arr = val_img_vec.reshape(img_h, img_w)
print type(val_img_arr)
val_img = Image.fromarray(val_img_arr)
print type(val_img)
val_img_c = cv2.cvtColor(val_img_arr, cv2.COLOR_BAYER_GR2RGB)

print "Saving image..."
cv2.imwrite("sample_output.png", val_img_c)
print "Done.\n"




# TASK 2
print "\n\nChecking for Point Clouds now...\n\n"

#get the messages of the entered topic
bag_pc_msg = bag.read_messages(topics=["/velodyne_points"])
print "The topic chosen: /velodyne_points \n"

#print a sample from the generator
print "\nChecking the a sample from the generator...\n"
topic, msg, time_stamp = bag_pc_msg.next()
print "The type of output from the generator is: ", type(bag_pc_msg.next()), "\n"
print "\nInside this item are the values: \n"
print type(topic), "\n",type(msg), "\n",type(time_stamp), "\n"
print "The string value: ", topic, "\n"

print "The attributes and methods associated with the second value: ", dir(msg), "\n"
print "The attributes and methods associated with the third value: ", dir(time_stamp), "\n"

print "The return type for the function:\tdata\tis:\t", type(msg.data), "\n"
print "The return type for the function:\tdeserialize\tis:\t", type(msg.deserialize), "\n"
print "The return type for the function:\tdeserialize_numpy\tis:\t", type(msg.deserialize_numpy), "\n"
print "The return type for the function:\tfields\tis:\t", type(msg.fields), "\n"
print "The return type for the function:\theader\tis:\t", type(msg.header), "\n"
print "The return type for the function:\theight\tis:\t", type(msg.height), "\n"
print "The return type for the function:\tis_bigendian\tis:\t", type(msg.is_bigendian), "\n"
print "The return type for the function:\tis_dense\tis:\t", type(msg.is_dense), "\n"
print "The return type for the function:\tpoint_step\tis:\t", type(msg.point_step), "\n"
print "The return type for the function:\trow_step\tis:\t", type(msg.row_step), "\n"
print "The return type for the function:\tserialize\tis:\t", type(msg.serialize), "\n"
print "The return type for the function:\tserialize_numpy\tis:\t", type(msg.serialize_numpy), "\n"
print "The return type for the function:\twidth\tis:\t", type(msg.width), "\n"
print "The return type for the function:\tdir(msg.data)\tis:\t", dir(msg.data), "\n"

print "The returned value for fields is: ", np.asarray(msg.fields), "\n"
print "The returned value for width is: ", msg.width, "\n"
print "The returned value for height is: ", msg.height, "\n"
print "The returned value for point_step is: ", msg.point_step, "\n"
print "The returned value for row_step is: ", msg.row_step, "\n"

fields = np.expand_dims(msg.fields, axis=0)
fields_short = ["x","y","z","intensity","ring"]
print "fields shape:", fields.shape
val_pc_vec = np.fromstring(msg.data, dtype=np.uint8)
print "The new value type is: ", type(val_pc_vec), "\nand the shape is: ",
val_pc_vec.shape, "\n", "value sample: ", val_pc_vec[:10], "...\n"

print "Reshape the vector back to the given filed headers:\n"
val_pc_arr = np.reshape(val_pc_vec,(-1,len(msg.fields)))
print "The new shape is: ", val_pc_arr.shape, "\nWith values: \n", np.vstack((fields_short, val_pc_arr))
val_pc_arr_headers = np.vstack((fields_short, val_pc_arr))
cwd = os.getcwd()
directory = cwd
print "Saving to directory/file: ", directory

# Save an array to a binary file in NumPy .npy format.
np.save(directory + "/X_Y_Z_intensity_ring", val_pc_arr_headers)

# Save as CSV file without the headers
# np.savetxt(directory + "/sample_point_cloud output.csv", val_pc_arr, delimiter=",")

# Save as CSV with headers using pandas, uncomment the import statement above to use
# data_f = pd.DataFrame(val_pc_arr)
# df.to_csv(directory, header=fields_short,index=False)

print "Saved."
