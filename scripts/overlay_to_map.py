#!/usr/bin/python

"""
This script overlays the gps points held within raw data
file (as prepared using pull_images_and_gps.py) or a csv file
for a jackal run onto a google map. It also generates
label files for the run(s) given. 

Usage for single file: ./overlay_to_map.py single [map_specs_file] [run folder] [bin size (m)]
    Parameters: 
        map_specs_file - A text file containing the specifications
                         for the map overlay in the following format:

                         map_img_name
                         bottom_right_corner_latitude;bottom_right_corner_longitude
                         top_left_corner_latitude;top_left_corner_longitude

        run folder - The path to the folder which contains the 'raw_data'
                        and 'preprocessed_data' sub-folders for a run. 
                        In other words, the folder which contains the data to 
                        map and label. 

        bin size - The size of the bins to be used for labelling the images 
                   to feed the NN for training. This should be specified in meters. 

Usage for multiple files: ./overlay_to_map.py multiple [map_specs_file] [parent_folder] [bin size(m)]
    Parameters:
        map_specs_file - Same as for single files. 
        
        parent_folder - The folder whose sub-folders are particular runs to overlay onto
                        the map. In other words, this is the parent_folder which contains
                        all of the run folders to process. 

        bin size - Same as for single files. 


Author: Rodolfo Corona, rcorona@utexas.edu
"""

import rospy
import rosbag
import sys
from geopy.distance import VincentyDistance
import matplotlib.pyplot as plt
from scipy.misc import imread
import matplotlib.cbook as cbook
from math import sqrt
import math
import numpy as np
import subprocess
import os

"""
Reads and returns coordinate lists from a raw
data file that was generated using 
pull_images_and_gps.py
"""
def coordinates_from_raw_data_file(label_file_name):    
    label_file = open(label_file_name, 'r')
    
    #Reads in data into lists. 
    lat_points = []
    long_points = []
    img_files = []

    for line in label_file:
        #Parses line for values. 
        values = line.split(';')

        latitude = float(values[0])
        longitude = float(values[1])
        img_file = values[2].strip()

        #If invalid gps data, then don't include data point.     
        if not (math.isnan(latitude) or math.isnan(longitude)):
            lat_points.append(latitude)
            long_points.append(longitude)
            img_files.append(img_file)
        
    label_file.close()

    return [lat_points, long_points, img_files]

"""
Reads in and returns map specifications
from a map specifications file. 
"""
def read_in_overlay_specs(overlay_specs_file):
    specs_file = open(overlay_specs_file, 'r')

    #Reads in specs and returns them.
    specs = []

    for line in specs_file:
        specs.append(line.strip())

    specs_file.close()

    #Formats specs and returns them. 
    map_img_name = specs[0]
    BR = tuple([float(value) for value in specs[1].split(';')])
    TL = tuple([float(value) for value in specs[2].split(';')])

    return [map_img_name, BR, TL]

"""
Takes the top left and bottom right corners of the coordinate plane
in gps coordinates as well as lists of latitude and longitude
points to translate into a coordinate system. (0,0) will map to
the bottom left corner of the image. 
"""
def get_coordinates_in_meters(TL, BR, lat_points, long_points):
    x_points = []
    y_points = []

    height = float(VincentyDistance((BR[0], TL[1]), (TL[0], TL[1])).meters)
    width = float(VincentyDistance((BR[0], TL[1]), (BR[0], BR[1])).meters)

    #Translates each coordinate point based on formula below. 
    for i in range(len(lat_points)):
        h = float(VincentyDistance((BR[0], TL[1]), (lat_points[i], long_points[i])).meters)
        e = float(VincentyDistance((BR[0], BR[1]), (lat_points[i], long_points[i])).meters)

        x = ((h ** 2) + (width ** 2) - (e ** 2)) / (2 * width)
        y = sqrt((h ** 2) - (x ** 2))

        x_points.append(x)
        y_points.append(y)

    return [x_points, y_points, width, height]

"""
Creates a grid given a square bin length along with 
the height and width of the total map plot for visualization. 
Returns a list of labels pertaining to the xy pairs given. 
These labels may be used for training. 
"""
def grid_and_labels_from_bin_size(bin_length, figure, width, height, x_points, y_points):
    #Rounds up for even dimmensions. 
    width = int(math.ceil(width))
    height = int(math.ceil(height))
    
    #Sets grid on plot for visualization. 
    axes = figure.gca()
    axes.set_xticks(np.arange(0, width, bin_length))
    axes.set_yticks(np.arange(0, height, bin_length))
    plt.grid()

    #Gets dimmensions for bin matrix representation. 
    num_columns = abs(width / bin_length) + (1 if width % bin_length else 0)
    num_rows = (height / bin_length) + (1 if height % bin_length else 0)

    #Gets labels for each xy pair pertaining to bin numbers.  
    labels = []

    for i in range(len(x_points)):
        column = int(x_points[i]) / bin_length
        row = int(y_points[i]) / bin_length

        """
        CHANGE THIS PART IF DIFFERENT LABELS ARE NEEDED. 

        Currently labels are real values in the following format:
        img;x;y
        """
        labels.append([x_points[i], y_points[i]])

    return labels
    
"""
Calls program to undistort all the images 
listed in the input parameter. The undistorted
images are put in an 'processed' folder
in the same parent directory as that of the
raw image folder, particularly in the
'preprocessed_data' folder.

The images will also be resized to 
360 x 224 in order to be valid
for NN training. 
"""
def preprocess_images(image_files, parent_folder):
    preprocessed_imgs = []

    #For keeping track of progress. 
    count = 1

    #Preprocess each image. 
    for image in image_files:
        #Prepares preprocessed image's name. 
        preprocessed_img = parent_folder + '/preprocessed_data/' + os.path.basename(image)    

        #Arguments for undistort binary. 
        args = ('undistort/undistort', 'undistort/cam_left.yml', image, preprocessed_img) 

        #Runs undistort program on image. 
        popen = subprocess.Popen(args, stdout=subprocess.PIPE)
        popen.wait()

        #Adds preprocessed image to list. 
        preprocessed_imgs.append(preprocessed_img)

        #Updates progress. 
        sys.stdout.write('\rProcessed image ' + str(count) + ' of ' + str(len(image_files)))
        sys.stdout.flush()
        count += 1

    #Clears progress tracking. 
    sys.stdout.write('\r')
    sys.stdout.flush()

    return preprocessed_imgs

"""
This creates a text file containing the
names of each (processed) image in a given
list along with their labels. The resulting
file may be used to construct the input
to a NN. 
"""
def create_label_file(labels, img_files, parent_folder):
    label_file = open(parent_folder + '/preprocessed_data/labels.txt', 'w')

    #Writes img/bin pairs to label file. 
    for i in range(len(img_files)):
        label_file.write(img_files[i] + ';' + str(labels[i][0]) + ';' + str(labels[i][1]) + '\n')

    label_file.close()

"""
Overlays a map onto gps coordinate points using
a map specification file and a file containing
gps coordinate points.

Also uses the created grid to assign labels to each
bin in the grid. All the points in the coordinate file
are then translated to bins. This list of pre-processed,
labelled data is returned. 
"""
def overlay_and_label(overlay_specs_file, parent_folder, bin_size):
    #Determines coordinates of corner of map image. 
    map_img_name, BR, TL = read_in_overlay_specs(overlay_specs_file)

    #Gets coordinate points from run. 
    lat_points, long_points, img_files = coordinates_from_raw_data_file(parent_folder + '/raw_data/raw_data.txt')

    #Ensures same number of latitude and longitude points were gathered. 
    if not len(lat_points) == len(long_points):
        print "latitude and longitude points disjoint in size!!!"

        sys.exit()

    #Translates them into coordinate system in meters. 
    x_points, y_points, width, height = get_coordinates_in_meters(TL, BR, lat_points, long_points)

    #Loads map image onto plot using corner coordinates. 
    figure = plt.figure()
    map_img_file = imread(map_img_name)
    plt.imshow(map_img_file, zorder=0, extent=[0.0, width, 0.0, height])

    #Plots coordinate points and shows plot. 
    plt.plot(x_points, y_points, 'ro')

    #Creates grid that will correspond to location bins to train NN.
    #Also generates labels associated with the bins. 
    labels = grid_and_labels_from_bin_size(int(bin_size), figure, width, height, x_points, y_points)

    #Pre-processes images (i.e. undistorts) and returns list of undestorted image names. 
    img_files = preprocess_images(img_files, parent_folder)

    #Ensures dimmensions are still correct. 
    if not len(img_files) == len(labels):
        print "# of imgs and # of labels does not match!"

        sys.exit()

    #Creates a label file for the data. 
    create_label_file(labels, img_files, parent_folder)

    #Presents the plot. 
#plt.show()

"""
Runs overlay_and_label over an entire folder. 
In other words, processes all the runs held
within a folder.

NOTE: This function assumes that every directory
within the specified parent_folder pertains to 
a run. Errors WILL occur if parent_folder contains
directories which do not pertain to a run and have
not been processed by pull_images_and_gps.py. 
"""
def overlay_and_label_folder(map_specs_file, parent_folder, bin_size):
    for entry in os.listdir(parent_folder):
        #If directory, then run overlaying procedure on it. 
        if os.path.isdir(parent_folder + '/' + entry):
            print 'Overlaying and labelling ' + parent_folder + '/' + entry + '...'
            overlay_and_label(map_specs_file, parent_folder + '/' + entry, bin_size)

if __name__ == "__main__":
    if not len(sys.argv) == 5:
        print 'Usage for single file: ./overlay_to_map.py single [map_specs_file] [run folder] [bin size (m)]' 
        print 'Usage for multiple files: ./overlay_to_map.py multiple [map_specs_file] [parent folder] [bin size(m)]'
    elif sys.argv[1] == 'single':
        overlay_and_label(sys.argv[2], sys.argv[3], sys.argv[4])
    elif sys.argv[1] == 'multiple':
        overlay_and_label_folder(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        print 'Usage for single file: ./overlay_to_map.py single [map_specs_file] [run folder] [bin size (m)]' 
        print 'Usage for multiple files: ./overlay_to_map.py multiple [map_specs_file] [parent folder] [bin size(m)]'

