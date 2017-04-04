# Didi Challenge Helper Scripts
Helper Scripts for the Didi Challenge (Team: Transformers!!)

## `rosbag_cooking.py`
* Extracts a sample image from a bag file in the same directory and prints out a summary and information about the bag file.
### Usage:
  * Open the file and rename 'intersection_1.bag' to the bag file name you want to use in the same directory as this file.
  * `python2 rosbag_cooking.py`

## `bag2images.py`
* Extracts all the images from a give bag file to a folder in the directory of a given name in grayscale or RGB colour.
### Usage:
  * Run from the command line as follows without brackets and parenthesis:
  * `python2 bag2images.py [bag_file_name] [new_images_folder_name] ["c" or "g" for colour or gray]`

## `bag2video.py`
* Extracts the images in the bag in video format in grayscale or RGB clolour.
### Usage:
  * Run from the command line as follows without brackets and parenthesis:
  * `python2 bag2video.py [file_name.bag] [fps] ["c" or "g" for colour or gray]`


### Also...

## `bag2csv.py`
* Converts the topics and types of the rosbag into csv format.
### Origin: [link to clearpathrobotics!](http://www.clearpathrobotics.com/downloads/support/bag_to_csv.zip)
