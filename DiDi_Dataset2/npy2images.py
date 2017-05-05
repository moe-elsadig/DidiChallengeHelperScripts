import numpy as np
import os
import matplotlib.pyplot as plt
import cv2
import matplotlib.image as mpimg
import argparse

# %matplotlib inline
cwd = os.getcwd()
file_name = "approach_1.bag_format_XYZIR.npy"

# Extraction function
def extract_cropped_images():
    #load all the frames
    print "Loading all frames..."

    #load all the frames in the given npy file
    all_frames = np.asarray(np.load(cwd + "/" + file_name))
    print "Frames loaded.\nExtracting images..."
    img_count = 0
    #iterate through the frames and limit all x and y values to the range -25 to 25
    for frame in all_frames:
        img_count += 1
        #Extract the points
        x_s = frame[0]
        y_s = frame[1]
        z_s = frame[2]
        i_s = frame[3]
        r_s = frame[4]

        #Filter on a height limit
        del_indeces = []
        del_count = 0

        #limit the points on the frame based on height (enough for pedestrians and vehicles)
        #create a list of indeces corresponding to points outside the height range
        # for i in range(len(z_s)):
        #     if z_s[i] > 0.8 or z_s[i] < -1.30:
        #         del_indeces.append(i-del_count)
        #         del_count += 1

        #delete the incdeces obtained above
        for index in del_indeces:
            del z_s[index], x_s[index], y_s[index], i_s[index], r_s[index]

        #Image setup, plot, and save as full frame under the bag file's name
        fig = plt.figure(frameon = False, figsize=(15,15))
        cm = plt.cm.get_cmap('Spectral')
        sc = plt.scatter(x_s, y_s, c=z_s, s=5, edgecolors= '', cmap=cm)
        # plt.axis('square')
        ax = plt.gca()
        ax.axes.get_xaxis().set_visible(False)
        ax.axes.get_yaxis().set_visible(False)
        ax.patch.set_facecolor('black')
        ax.set_xlim([-30,30])
        ax.set_ylim([-30,30])
        # plt.colorbar(sc)

        #directory for the images to be saved in
        directory = cwd + "/" + file_name[:5] + "extraction"
        if not os.path.exists(directory):
            os.makedirs(directory)

        plt.savefig((directory + "/img_" + str(img_count) + ".png"), bbox_inches='tight', dpi=300, pad_inches=0.0)

        #Load the image again for cropping
        croppable_img = cv2.imread(directory + "/img_" + str(img_count) + ".png")

        #ysize
        ysize = croppable_img.shape[0]
        #xsize
        xsize = croppable_img.shape[1]
        #step size
        step = 230
        #second run offset
        offset = 115

        cropped_img_count = 0
        for x_step in range(np.int(xsize/step)):
            for y_step in range(np.int(ysize/step)):
                cropped_img_count += 1
                cropped_img = croppable_img[step*x_step:step*x_step+step , step*y_step:step*y_step+step]
                imgdirectory = directory + "/frame_" + str(img_count)
                if not os.path.exists(imgdirectory):
                    os.makedirs(imgdirectory)
                cv2.imwrite((imgdirectory + "/cropped_img_" + str(cropped_img_count) + "_frame_" + str(img_count) + ".png"), cropped_img)

        cropped_img_count = 0
        for x_step in range(np.int(xsize/step)):
            for y_step in range(np.int(ysize/step)):
                cropped_img_count += 1
                cropped_img = croppable_img[offset+step*x_step:offset+step*x_step+step , offset+step*y_step:offset+step*y_step+step]
                imgdirectory = directory + "/frame_" + str(img_count)
                if not os.path.exists(imgdirectory):
                    os.makedirs(imgdirectory)
                cv2.imwrite((imgdirectory + "/cropped_img_" + str(cropped_img_count) + "_frame_wo_" + str(img_count) + ".png"), cropped_img)

    print "Extraction complete.\n"
    print "The number of frames Extracted, Converted then Cropped are: ", img_count, "\n"
    print "Done."

parser = argparse.ArgumentParser(description='Numpy File: Cropped Images from velodyne_points.npy')
parser.add_argument(
	'npy',
	type=str,
	help='Path to rosbag file'
)
args = parser.parse_args()
file_name = args.npy

extract_cropped_images()
