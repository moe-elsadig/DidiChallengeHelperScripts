# Didi Challenge Helper Scripts
Helper Scripts for the Didi Challenge (Team: Transformers!!)

> These Scripts use Python2.x

# Dataset1


## `rosbag_cooking.py`
* Extracts a sample image from a bag file in the same directory and prints out a summary and information about the bag file.
* Prints a lot of information on the file's PointCloud2 Topic and a sample of the data to terminal.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
`python2 rosbag_cooking.py [file_name.bag]`

## `bag2images.py`
* Extracts all the images from a given bag file to a folder in the directory of a given name in grayscale or RGB colour.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 bag2images.py [bag_file_name] [new_images_folder_name] ["c" or "g" for colour or gray]`
![alt tag](https://github.com/moe-elsadig/DidiChallengeHelperScripts/blob/master/sample_output.png)

## `bag2video.py`
* Extracts the images in the bag in video format in grayscale or RGB clolour.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 bag2video.py [file_name.bag] [fps] ["c" or "g" for colour or gray]`

## `bag2pointcloud_xyzir.py`
* Extracts the PointCloud2 frames in the bag file as an array of frames(topic messages).
* Each member of the output array contains an array of the X - Y - Z - Intensity-Ring values in this order.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
`python2 bag2pointcloud_xyzir.py [file_name.bag]`
* The output file can be loaded using:
`numpy.load([npy_file_path])`

## `pointcloud2plot_visual.ipynb`
* Extracts the PointCloud2 frames in the `.NPY` file as an array of frames values (X,Y,Z,Intensity,Ring).
* Plots a bird's-eye view of a sample frame and saves it to an image at 1400dpi.
* **Note!** VTK and MayaVI need to be on compatible versions with each other.

## `bag2velo_xy_images.py`
* Extracts all the xy axis (bird's-eye view) images from a given bag file to a folder in the directory.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 bag2velo_xy_images.py [bag_file_name] [int_dpi]`

## `Feature_extraction_from_velo_points.ipynb`
* A Notebook exploring how to extract features from the velodyne data provided

## `npy2images.py`
* Extracts all the points form the `.NPY` file, converts them to top-down images, saves the images and a folder of their image cropped into chuncks of 230x230 to manually label the car/notCar data for Classification purposes
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 npy2images.py [npy_file_name]`

![alt tag](https://github.com/moe-elsadig/DidiChallengeHelperScripts/blob/master/2d_plot.png)


# Dataset2

## `rosbag_cooking2.py`
* Extracts a sample image from a bag file in the same directory and prints out a summary and information about the bag file.
* Prints a lot of information on the file's PointCloud2 Topic and a sample of the data to terminal.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
`python2 rosbag_cooking2.py [file_name.bag]`


## `bag2images.py`
* Extracts all the images from a given bag file to a folder in the directory of a given name in grayscale or RGB colour.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 bag2images.py [bag_file_name] [new_images_folder_name] ["c" or "g" for colour or gray]`
![alt tag](https://github.com/moe-elsadig/DidiChallengeHelperScripts/blob/master/sample_output.png)


## `bag2pointcloud_xyzir.py`
> NOTE: for Dataset2 bag file, you must first use the conversion technique below to convert from velodyne_packets in the available bag file to bag files with point_cloud2

* Extracts the PointCloud2 frames in the bag file as an array of frames(topic messages).
* Each member of the output array contains an array of the [X - Y - Z - Intensity - Ring] values in this order.


### Usage:
* Run from the command line as follows without brackets and parenthesis:
`python2 bag2pointcloud_xyzir.py [file_name.bag]`
* The output file can be loaded using:
`numpy.load([npy_file_path])`


## `npy2images.py`
* Extracts all the points form the `.NPY` file, converts them to top-down images, saves the images and a folder of their image cropped into chuncks of 230x230 to manually label the car/notCar data for Classification purposes
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 npy2images.py [npy_file_name]`

## `tinynpy2images.py`
* Extracts all the points form the `.NPY` file, converts them to top-down images, saves the images and a folder of their image cropped into chuncks of 230x230 to manually label the car/notCar data for Classification purposes
* This is useful for using a method outlined by the Captain of the Transformers for Image-by-Image extraction.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 tinynpy2images.py [npy_file_name]`

## `the_extractor_2.py`
* An attempt at extracting all the available topics and data into npy, csv, text formats for the sake of simplicity or study.
### Usage:
* Run from the command line as follows without brackets and parenthesis:
  `python2 the_extractor_2.py [bag_file_name]`

> Currently Supports:
> -extract_tracks()
> -extract_steering_report()
> -extract_brake_report()
> -extract_twist()
> -extract_objects_gps_fix()
> -extract_wheel_speed_report()
> -extract_objects_gps_rtkfix()
> -extract_time()

### Also Checkout...

## `bag_to_csv`
* Converts the topics and types of the rosbag into csv format.
### Origin: [link to clearpathrobotics!](http://www.clearpathrobotics.com/downloads/support/bag_to_csv.zip)
