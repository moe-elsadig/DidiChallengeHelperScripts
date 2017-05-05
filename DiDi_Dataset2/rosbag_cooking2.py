import rosbag
import yaml
import numpy as np
from PIL import Image
import cv2
import os
# import pandas as pd
import argparse
import sensor_msgs.point_cloud2 as pc2

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


#function to get the type and topics information from the bag
topics = bag.get_type_and_topic_info()[1].keys()

#Iterate over the topics and return a list of the types
types = []
for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
    types.append(bag.get_type_and_topic_info()[1].values()[i][0])

print"The available topics are:\n"
for i in range(len(topics)):
    print"topic " , i , " " , topics[i]

print"\n"




count = 0

print"Checking topic ", count, "\n"
#get the messages of the entered topic
full_msg = bag.read_messages(topics=[topics[count]])
print "The topic chosen: ", topics[count], "\n"

print type(full_msg)

# print type(full_msg.next())

topic, msg, time_stamp = full_msg.next()

print type(msg)

print (msg.width)












#end
