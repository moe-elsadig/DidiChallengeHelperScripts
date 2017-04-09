import rosbag
import numpy as np
from PIL import Image
import cv2
import argparse
import os

rosbag_file = None
cwd = os.getcwd()
color = "c"
fps = 30

def extract_video():

    bag = rosbag.Bag((rosbag_file), 'r')
    bag_img_msg = bag.read_messages(topics=["/image_raw"])

    print "Processing images to video..."

    # SETTINGS FOR SAVING OUTPUT VIDEO
    out_file = cwd + "/" + rosbag_file[:-4] + "_video.avi"  # Filepath to save the video as
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
    print "Done.\nVideo saved to", cwd + "/..."

# if __name__ == '__main__':
parser = argparse.ArgumentParser(description='Kalman Filter Tracker')
parser.add_argument(
	'bag',
	type=str,
	help='Path to rosbag file'
)
parser.add_argument(
	'fps',
	type=int,
	help='Frames Per Second'
)
parser.add_argument(
	'color',
	type=str,
	help='Image colour type'
)

args = parser.parse_args()
rosbag_file = args.bag
color = args.color
fps = args.fps

extract_video()
# print dir(cv2)
