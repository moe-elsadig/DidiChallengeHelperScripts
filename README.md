# Didi Challenge Helper Scripts
Helper Scripts for the Didi Challenge (Team: Transformers!!)

> These Scripts use Python2.x
> New Addition: bag2velo_xy_images.py
> New Addition: PointCloud2Plotter.ipynb
> New Addition: bag2pointcloud_xyzir.py

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



### Also Checkout...

## `bag_to_csv`
* Converts the topics and types of the rosbag into csv format.
### Origin: [link to clearpathrobotics!](http://www.clearpathrobotics.com/downloads/support/bag_to_csv.zip)
