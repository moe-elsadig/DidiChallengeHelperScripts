import rosbag
import numpy as np
from PIL import Image
import cv2
import argparse
import os

rosbag_file = None
folder_name = "images_folder"
color = "c"
cwd = os.getcwd()

def extract_images():

    bag = rosbag.Bag((rosbag_file), 'r')
    bag_img_msg = bag.read_messages(topics=["/image_raw"])
    count = 0
    print "Saving image..."

    for topic, msg, time in bag_img_msg:
        img_w = msg.width
        img_h = msg.height
        val_img_vec = np.fromstring(msg.data, dtype=np.uint8)
        val_img_arr = val_img_vec.reshape(img_h, img_w)
        directory = cwd + "/" + folder_name + "_" + "/images" + color
        if not os.path.exists(directory):
            os.makedirs(directory)
        if (color == "g"):
            cv2.imwrite((directory + "/img_gray_" + str(count) + ".png"), val_img_arr)
        else:
            val_img_c = cv2.cvtColor(val_img_arr, cv2.COLOR_BAYER_GR2RGB)
            cv2.imwrite((directory + "/img_colour_" + str(count) + ".png"), val_img_c)

        count += 1

    print "Done.\n", count, " Images saved to", cwd + "/" + folder_name + "_" + color + "/..."



def extract_video():

    directory = cwd + "/" + folder_name + "_" + "/video_" + str(fps)
    if not os.path.exists(directory):
        os.makedirs(directory)
    bag = rosbag.Bag((rosbag_file), 'r')
    bag_img_msg = bag.read_messages(topics=["/image_raw"])

    print "Processing images to video..."

    # SETTINGS FOR SAVING OUTPUT VIDEO
    out_file = directory + "/" + rosbag_file[:-4] + "_video.avi"  # Filepath to save the video as
    fourcc = cv2.cv.CV_FOURCC(*'XVID')
    out_vid_dims = (1400, 512)
    fps_ = fps  # adjust based on input video

    coloring = False
    if color == "c":
        coloring = True

    out = cv2.VideoWriter(out_file,
                          fourcc=fourcc,
                          fps=fps_,
                          frameSize=out_vid_dims,
                          isColor=coloring)

    for topic, msg, time in bag_img_msg:
        img_w = msg.width
        img_h = msg.height
        val_img_vec = np.fromstring(msg.data, dtype=np.uint8)
        val_img_arr = val_img_vec.reshape(img_h, img_w)

        if (color == "g"):
            out.write(val_img_arr)
        else:
            val_img_c = cv2.cvtColor(val_img_arr, cv2.COLOR_BAYER_GR2RGB)
            out.write(val_img_c)



    out.release()
    print "Done.\nVideo saved to", directory + "/..."























# if __name__ == '__main__':
parser = argparse.ArgumentParser(description='ROSBAG File: bag2images')
parser.add_argument(
	'bag',
	type=str,
	help='Path to rosbag file'
)

parser.add_argument(
	'folder_name',
	type=str,
	help='Name of output folder'
)

parser.add_argument(
	'color',
	type=str,
	help='Image colour type'
)

parser.add_argument(
	'fps',
	type=int,
	help='Frames Per Second'
)

args = parser.parse_args()
rosbag_file = args.bag
folder_name = args.folder_name
color = args.color
fps = args.fps

extract_images()
extract_video()
