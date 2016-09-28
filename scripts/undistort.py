#!/usr/bin/python

"""
This script undistorts all of the images in
a folder and writes an undistorted version
of each one in a desired folder.

Usage
------

For undistorting all images in a folder: ./undistort.py folder [input_folder] [output_folder] [camera]

    Parameters:

        input_folder            - The folder containing the images to undistort. 

        output_folder           - The folder in which to place the undistorted version
                                  of each image. 

        camera                  - The camera which took the images (left or right).


Author: Rodolfo Corona, rcorona@utexas.edu
"""

import os
import sys
import subprocess

#Path to undistort executable, should be in build folder.  TODO Change if needed. 
undistort_path = '/home/rcorona/catkin_ws/src/particle_filter/src/undistort/build/'

def undistort_folder(input_folder, output_folder, camera):
    if camera == 'left':
        camera_yml = undistort_path + 'cam_left.yml'
    elif camera == 'right':
        camera_yml = undistort_path + 'cam_right.yml'
    else:
        print 'Invalid camera name entered! (try "right" or "left")'
        sys.exit()

    #Goes throuh each file in folder. 
    for file_name in os.listdir(input_folder):
        name, extension = file_name.split('.')

        #If file is jpg, then attempt to undistort it. 
        if extension == 'jpg':
            #Prepares arguments for calling undistortion executable and calls it.
            args = [undistort_path + 'undistort', camera_yml, input_folder + '/' + file_name, output_folder + '/' + file_name]
            print args
            subprocess.call(args)

            print 'Undistorted ' + file_name

"""
Prints the usage 
instructions for 
the script. 
"""
def print_usage():
    print 'Undistort folder: ./undistort.py folder [input_folder] [output_folder] [camera]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'folder':
        if len(sys.argv) == 5:
            undistort_folder(sys.argv[2], sys.argv[3], sys.argv[4])
        else:
            print_usage()
   
    else:
        print_usage()
