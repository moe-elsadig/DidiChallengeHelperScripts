# topic  0   /radar/tracks
# topic  1   /vehicle/steering_report
# topic  2   /vehicle/brake_report
# topic  3   /vehicle/twist
# topic  4   /objects/capture_vehicle/front/gps/fix
# topic  5   /vehicle/wheel_speed_report
# topic  6   /objects/obs1/rear/gps/rtkfix
# topic  7   /vehicle/gps/time
# topic  8   /cloud_nodelet/parameter_descriptions
# topic  9   /vehicle/gps/vel
# topic  10   /vehicle/gps/fix
# topic  11   /vehicle/suspension_report
# topic  12   /vehicle/sonar_cloud
# topic  13   /objects/capture_vehicle/front/gps/time
# topic  14   /velodyne_packets
# topic  15   /objects/obs1/rear/gps/fix
# topic  16   /diagnostics
# topic  17   /objects/capture_vehicle/rear/gps/rtkfix
# topic  18   /vehicle/brake_info_report
# topic  19   /vehicle/imu/data_raw
# topic  20   /vehicle/surround_report
# topic  21   /vehicle/gear_report
# topic  22   /radar/range
# topic  23   /objects/capture_vehicle/rear/gps/fix
# topic  24   /vehicle/throttle_report
# topic  25   /image_raw
# topic  26   /vehicle/fuel_level_report
# topic  27   /rosout
# topic  28   /velodyne_nodelet_manager/bond
# topic  29   /vehicle/filtered_accel
# topic  30   /vehicle/joint_states
# topic  31   /vehicle/twist_controller/parameter_descriptions
# topic  32   /cloud_nodelet/parameter_updates
# topic  33   /diagnostics_toplevel_state
# topic  34   /objects/capture_vehicle/rear/gps/time
# topic  35   /objects/obs1/rear/gps/time
# topic  36   /can_bus_dbw/can_rx
# topic  37   /vehicle/twist_controller/parameter_updates
# topic  38   /vehicle/throttle_info_report
# topic  39   /vehicle/misc_1_report
# topic  40   /radar/points
# topic  41   /vehicle/tire_pressure_report
# topic  42   /diagnostics_agg
# topic  43   /objects/capture_vehicle/front/gps/rtkfix
# topic  44   /vehicle/dbw_enabled

import rosbag
import yaml
import numpy as np
from PIL import Image
import cv2
import os
import csv
# import pandas as pd
import argparse
import sensor_msgs.point_cloud2 as pc2

cwd = os.getcwd()

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

#function to get the type and topics information from the bag
topics = bag.get_type_and_topic_info()[1].keys()

def extract_tracks():

	print "Extracting tracks...\n"
	full_msg = bag.read_messages(topics=[topics[0]])

	tracks = []
	frame_header = ["status", "number", "range_", "rate", "accel", "angle", "width",
	"late_rate", "moving", "power", "absolute_rate"]

	for topic, msg, time in full_msg:
		for msg1 in msg.tracks:
			status = msg1.status
			number = msg1.number
			range_ = msg1.range
			rate = msg1.rate
			accel = msg1.accel
			angle = msg1.angle
			width = msg1.width
			late_rate = msg1.late_rate
			moving = msg1.moving
			power = msg1.power
			absolute_rate = msg1.absolute_rate

			frame = [status, number, range_, rate, accel, angle, width, late_rate, moving, power, absolute_rate]

			tracks.append(frame)


	print "Extracting Tracks complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/radar/"
	if not os.path.exists(directory):
		os.makedirs(directory)


	with open(directory + 'tracks.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()
		for track in tracks:
			writer.writerow({"status":track[0], "number":track[1],
            "range_":track[2], "rate":track[3], "accel":track[4], "angle":track[5],
            "width":track[6], "late_rate":track[7], "moving":track[8], "power":track[9],
            "absolute_rate":track[10]})

	print "Tracks file saved."


def extract_steering_report():

	print "Extracting steering_report...\n"

	full_msg = bag.read_messages(topics=[topics[1]])

	steering_report = []

	frame_header = ["steering_wheel_angle", "steering_wheel_angle_cmd", "steering_wheel_torque",
	"speed", "enabled", "override", "driver", "fault_wdc", "fault_bus1", "fault_bus2", "fault_calibration",
	"fault_connector"]

	for topic, msg, time in full_msg:
		steering_wheel_angle = msg.steering_wheel_angle
		steering_wheel_angle_cmd = msg.steering_wheel_angle_cmd
		steering_wheel_torque = msg.steering_wheel_torque
		speed = msg.speed
		enabled = msg.enabled
		override = msg.override
		driver = msg.driver
		fault_wdc = msg.fault_wdc
		fault_bus1 = msg.fault_bus1
		fault_bus2 = msg.fault_bus2
		fault_calibration = msg.fault_calibration
		fault_connector = msg.fault_connector

		frame = [steering_wheel_angle, steering_wheel_angle_cmd, steering_wheel_torque,
		speed, enabled, override, driver, fault_wdc, fault_bus1, fault_bus2, fault_calibration,
		fault_connector]

		steering_report.append(frame)

	print "Extracting steering_report complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/vehicle/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'steering_report.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()

		for frame in steering_report:

			writer.writerow({"steering_wheel_angle":frame[0],
			"steering_wheel_angle_cmd":frame[1], "steering_wheel_torque":frame[2],
			"speed":frame[3], "enabled":frame[4], "override":frame[5], "driver":frame[6],
			"fault_wdc":frame[7], "fault_bus1":frame[8], "fault_bus2":frame[9],
			"fault_calibration":frame[10],
			"fault_connector":frame[11]})

	print "steering_report file saved."


def extract_brake_report():

	print "Extracting brake_report...\n"

	full_msg = bag.read_messages(topics=[topics[2]])

	brake_report = []

	frame_header = ["pedal_input", "pedal_cmd", "pedal_output", "torque_input", "torque_cmd",
	"torque_output", "boo_input", "boo_cmd", "boo_output", "enabled", "override", "driver",
	"watchdog_braking", "fault_wdc", "fault_ch1", "fault_ch2", "fault_boo", "fault_connector"]

	for topic, msg, time in full_msg:
		pedal_input = msg.pedal_input
		pedal_cmd = msg.pedal_cmd
		pedal_output = msg.pedal_output
		torque_input = msg.torque_input
		torque_cmd = msg.torque_cmd
		torque_output = msg.torque_output
		boo_input = msg.boo_input
		boo_cmd = msg.boo_cmd
		boo_output = msg.boo_output
		enabled = msg.enabled
		override = msg.override
		driver = msg.driver
		watchdog_braking = msg.watchdog_braking
		fault_wdc = msg.fault_wdc
		fault_ch1 = msg.fault_ch1
		fault_ch2 = msg.fault_ch2
		fault_boo = msg.fault_boo
		fault_connector = msg.fault_connector

		frame = [pedal_input, pedal_cmd, pedal_output, torque_input, torque_cmd,
		torque_output, boo_input, boo_cmd, boo_output, enabled, override, driver,
		watchdog_braking, fault_wdc, fault_ch1, fault_ch2, fault_boo, fault_connector]

		brake_report.append(frame)

	print "Extracting brake_report complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/vehicle/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'brake_report.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()

		for frame in brake_report:
			writer.writerow({"pedal_input":frame[0], "pedal_cmd":frame[1],
			"pedal_output":frame[2], "torque_input":frame[3], "torque_cmd":frame[4],
			"torque_output":frame[5], "boo_input":frame[6], "boo_cmd":frame[7],
			"boo_output":frame[8], "enabled":frame[9], "override":frame[10], "driver":frame[11],
			"watchdog_braking":frame[12], "fault_wdc":frame[13],
			"fault_ch1":frame[14], "fault_ch2":frame[15], "fault_boo":frame[16],
			"fault_connector":frame[17]})

	print "brake_report file saved."


def extract_twist():
	print "Extracting twist...\n"

	full_msg = bag.read_messages(topics=[topics[3]])

	twist = []

	frame_header = ["linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"]

	for topic, msg, time in full_msg:
		linear_x = msg.twist.linear.x
		linear_y = msg.twist.linear.y
		linear_z = msg.twist.linear.z
		angular_x = msg.twist.angular.x
		angular_y = msg.twist.angular.y
		angular_z = msg.twist.angular.z

		frame = [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]

		twist.append(frame)

	print "Extracting twist complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/vehicle/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'twist.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()

		for frame in twist:
			writer.writerow({"linear_x":frame[0], "linear_y":frame[1],
			"linear_z":frame[2], "angular_x":frame[3],
			"angular_y":frame[4], "angular_z":frame[5]})

	print "twist file saved."


def extract_objects_gps_fix():
	print "Extracting gps_fix...\n"

	full_msg = bag.read_messages(topics=[topics[4]])

	gps_fix = []

	frame_header = ["status", "service", "latitude", "longitude", "altitude",
	"position_covariance", "position_covariance_type"]

	for topic, msg, time in full_msg:
		status = msg.status.status
		service = msg.status.service
		latitude = msg.latitude
		longitude = msg.longitude
		altitude = msg.altitude
		position_covariance = msg.position_covariance
		position_covariance_type = msg.position_covariance_type


		frame = [status, service, latitude, longitude, altitude, position_covariance,
		position_covariance_type]

		gps_fix.append(frame)

	print "Extracting gps_fix complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/objects/capture_vehicle/front/gps/fix/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'fix.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()

		for frame in gps_fix:
			writer.writerow({"status":frame[0], "service":frame[1],
			"latitude":frame[2], "longitude":frame[3], "altitude":frame[4],
			"position_covariance":frame[5], "position_covariance_type":frame[6]})

	print "gps_fix file saved."


def extract_wheel_speed_report():
	print "Extracting wheel_speed_report...\n"

	full_msg = bag.read_messages(topics=[topics[5]])

	wheel_speed_report = []

	frame_header = ["front_left", "front_right", "rear_left", "rear_right"]

	for topic, msg, time in full_msg:
		front_left = msg.front_left
		front_right = msg.front_right
		rear_left = msg.rear_left
		rear_right = msg.rear_right

		frame = [front_left, front_right, rear_left, rear_right]

		wheel_speed_report.append(frame)

	print "Extracting wheel_speed_report complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/vehicle/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'wheel_speed_report.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()

		for frame in wheel_speed_report:
			writer.writerow({"front_left":frame[0], "front_right":frame[1],
			"rear_left":frame[2], "rear_right":frame[3]})

	print "wheel_speed_report file saved."


def extract_objects_gps_rtkfix():
	print "Extracting gps_rtkfix...\n"

	full_msg = bag.read_messages(topics=[topics[6]])

	gps_rtkfix = []

	frame_header = ["pose_posx", "pose_posy", "pose_posz", "pose_orix", "pose_oriy", "pose_oriz", "pose_oriw",
	"pose_covariance",
	"twist_posx", "twist_posy", "twist_posz", "twist_orix", "twist_oriy", "twist_oriz", "twist_covariance"]

	for topic, msg, time in full_msg:
		pose_posx = msg.pose.pose.position.x
		pose_posy = msg.pose.pose.position.y
		pose_posz = msg.pose.pose.position.z
		pose_orix = msg.pose.pose.orientation.x
		pose_oriy = msg.pose.pose.orientation.y
		pose_oriz = msg.pose.pose.orientation.z
		pose_oriw = msg.pose.pose.orientation.w
		pose_covariance = msg.pose.covariance
		twist_posx = msg.twist.twist.linear.x
		twist_posy = msg.twist.twist.linear.y
		twist_posz = msg.twist.twist.linear.z
		twist_orix = msg.twist.twist.angular.x
		twist_oriy = msg.twist.twist.angular.y
		twist_oriz = msg.twist.twist.angular.z
		twist_covariance = msg.twist.covariance

		frame = [pose_posx, pose_posy, pose_posz, pose_orix, pose_oriy, pose_oriz, pose_oriw, pose_covariance,
		twist_posx, twist_posy, twist_posz, twist_orix, twist_oriy, twist_oriz, twist_covariance]

		gps_rtkfix.append(frame)

	print "Extracting gps_rtkfix complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/objects/obs1/rear/gps/rtkfix/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'pose.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header[:8], delimiter = ';')
		writer.writeheader()

		for frame in gps_rtkfix:
			writer.writerow({"pose_posx":frame[0], "pose_posy":frame[1],
			"pose_posz":frame[2], "pose_orix":frame[3],
			"pose_oriy":frame[4], "pose_oriz":frame[5],
			"pose_oriw":frame[6], "pose_covariance":frame[7]})

	with open(directory + 'twist.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header[8:], delimiter = ';')
		writer.writeheader()

		for frame in gps_rtkfix:
			writer.writerow({"twist_posx":frame[8], "twist_posy":frame[9],
			"twist_posz":frame[10], "twist_orix":frame[11],
			"twist_oriy":frame[12], "twist_oriz":frame[13], "twist_covariance":frame[14]})

	print "gps_rtkfix files saved."


def extract_time():
	print "Extracting time...\n"

	full_msg = bag.read_messages(topics=[topics[7]])

	time_ = []

	frame_header = ["secs", "nsecs"]

	for topic, msg, time in full_msg:
		secs = msg.time_ref.secs
		nsecs = msg.time_ref.nsecs

		frame = [secs, nsecs]

		time_.append(frame)

	print "Extracting time complete\n"

	directory = cwd + "/" + rosbag_file[:-4] + "/vehicle/gps/"
	if not os.path.exists(directory):
		os.makedirs(directory)

	with open(directory + 'time.csv', 'wb') as outcsv:
		writer = csv.DictWriter(outcsv, fieldnames = frame_header, delimiter = ';')
		writer.writeheader()

		for frame in time_:
			writer.writerow({"secs":frame[0], "nsecs":frame[1]})

	print "time file saved."




extract_tracks()
extract_steering_report()
extract_brake_report()
extract_twist()
extract_objects_gps_fix()
extract_wheel_speed_report()
extract_objects_gps_rtkfix()
extract_time()


# End
