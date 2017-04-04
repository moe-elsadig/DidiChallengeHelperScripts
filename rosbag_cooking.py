import rosbag
import yaml
import numpy as np
from PIL import Image
import cv2

#select the rosbag file you wish to check
bag = rosbag.Bag('intersection_1.bag', 'r')

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

print "\nThe topics in this file are: \n"
for i in range(len(types)):
    print i, ": ", types[i]
print "\n\n"

#get the messages of the entered topic
bag_img_msg = bag.read_messages(topics=["/image_raw"])
print "The topic chosen: /image_raw \n"

#check the type of the returned messages
print "The type of the returned message is: ", type(bag_img_msg), "\n"

#print a sample from the generator
print "\nChecking the a sample from the generator...\n"
topic, msg, time = bag_img_msg.next()
print "The type of output from the generator is: ", type(bag_img_msg.next()), "\n"
print "\nInside this item are the values: \n"
print type(topic), "\n",type(msg), "\n",type(time), "\n"
print "The string value: ", topic, "\n"

print "The attributes and methods associated with the second value: ", dir(msg), "\n"
print "The attributes and methods associated with the third value: ", dir(time), "\n"

img_w = msg.width
img_h = msg.height
print "Testing methods for value 2:\nwidth\n", img_w, "\nheight\n", img_h,"\n"

print "The values data type is: ", type(msg.data), "\n"
print msg.data[:20], "...\nunknown string needs to be coverted to int value using numpy\n"

val_img_vec = np.fromstring(msg.data, dtype=np.uint8)
print "The new value is: ", val_img_vec[:20], "...\nand the shape is: ", val_img_vec.shape, "\n"

print "Reshape the vector back to the given height and width using PIL and convert to IMG:\n"

val_img_arr = val_img_vec.reshape(img_h, img_w)
print type(val_img_arr)
val_img = Image.fromarray(val_img_arr)
print type(val_img)
val_img_c = cv2.cvtColor(val_img_arr, cv2.COLOR_BAYER_GR2RGB)

print "Saving image..."
cv2.imwrite("sample_output.png", val_img_c)
print "Done.\n"
