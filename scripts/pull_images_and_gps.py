#!/usr/bin/python

"""
This script takes a bag file and creates sets of images
labeled with their corresponding gps coordinates. 
The resulting file MUST still be pre-processed
in order to get class labels. The images must also 
be preprocessed to remove distortions and get them
to fit the required dimmensions. 
The images are pulled from the bag file every 'rate'
meters of movement. 

Usage for single file: ./pull_images_and_gps.py single [bag_file_name.bag] [rate]
    Parameters: 
        bag_file_name: The name of the bag file to pull from. 
        rate: The rate in meters to pull (i.e. sample) images in. 

Usage for multiple files: ./pull_images_and_gps.py multiple [folder] [rate]
    Parameters:
        folder: The folder which contains the bag files to be processed. 
        rate: The rate in meters to pull (i.e. sample) images in. 
            


Author: Rodolfo Corona, rcorona@utexas.edu
"""

import rospy
import rosbag
import sys
from math import sqrt
import os

"""
Prepares the data for the image using the latest values 
in the bag file. 
"""
def write_data(camera_folder, data_file, latest_values):
    #Writes image to jpg file in pertaining camera's folder. 
    img_file_name = camera_folder + str(latest_values['img_counter']) + '.jpg'
    img_file = open(img_file_name, 'w')

    img_file.write(latest_values['img'].data)

    img_file.close()

    #Writes images label to the label file.
    data = str(latest_values['gps'].latitude) + ';'
    data += str(latest_values['gps'].longitude) + ';'
    data += img_file_name + '\n'

    data_file.write(data)

"""
Creates the folder structure that will be used
to keep track of the raw and pre-processed
data for a run. 
"""
def create_folder_structure(top_level_folder_name):
    #Creates top level if needed. 
    if not os.path.isdir(top_level_folder_name):
        os.mkdir(top_level_folder_name)

    #Creates folder for raw data if needed. 
    if not os.path.isdir(top_level_folder_name + '/raw_data'):
        os.mkdir(top_level_folder_name + '/raw_data')

    #Creates folder for pre-processed data if needed. 
    if not os.path.isdir(top_level_folder_name + '/preprocessed_data'):
        os.mkdir(top_level_folder_name + 'preprocessed_data')

"""
The main method used to do the pulling. This is called from the main function,
but may be called seperately if being used in another script. 

The parameters correspond exactly to the ones specified at the top of this file (under Usage). 
"""
def pull_every_n_meters(file_name, rate, camera):
    #Creates a folder name for the processing of the file. 
    camera_folder = file_name.split('.')[0] + '/' 

    #Opens bag file.
    bag_file = rosbag.Bag(file_name, 'r')

    #Creates folder structure if it does not yet exist. 
    create_folder_structure(camera_folder)

    #Label files to build training set.
    data_file = open(camera_folder + 'raw_data/raw_data.txt', 'w')

    #Topic names. 
    left_camera = '/camera_left/image_color/compressed'
    right_camera = '/camera_right/image_color/compressed'
    camera_topic = left_camera if camera == 'left' else right_camera
    
    odom = '/jackal_velocity_controller/odom'
    gps = '/navsat/fix'

    #Converts to float for use in calculations. 
    rate = float(rate)

    #Keeps track of pertinent data. 
    odom_values = {'prev_x': 0.0, 'prev_y': 0.0, 'diff': 0.0}
    latest_values = {
                     'img': None, 
                     'gps': None,
                     'img_counter': 0
                    }

    #Reads bag file to pull images. 
    for topic, msg, t in bag_file.read_messages():
        #First three topics merely update their respective reading. 
        if topic == camera_topic:
            latest_values['img'] = msg
        elif topic == gps:
            latest_values['gps'] = msg
        #If odometry, then check if movement rate has been met.
        elif topic == odom:
            #Calculates movement total since last reading. 
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)

            dx = odom_values['prev_x'] - x
            dy = odom_values['prev_y'] - y

            movement = sqrt((dx ** 2) + (dy ** 2))
            odom_values['diff'] += movement

            #Sets prev values to new values. 
            odom_values['prev_x'] = x
            odom_values['prev_y'] = y

            #If rate distance has been reached, pull image and reset counter. 
            if odom_values['diff'] >= rate:
                odom_values['diff'] = 0.0
        
                #Writes image label pairs for left and right cameras, updates their counters.  
                if not (latest_values['img'] == None or latest_values['gps'] == None):
                    write_data(camera_folder + 'raw_data/', data_file, latest_values)
                    latest_values['img_counter'] += 1

    #Closes files. 
    data_file.close()
    bag_file.close()

"""
This function processes an entire folder of bag files
using the desired pulling specifications. 
"""
def process_folder(folder, rate, camera):
    for entry in os.listdir(folder):
        #If bag file, then process. 
        if entry.endswith('.clipped') or entry.endswith('.bag'):
            print 'Pulling from ' + folder + '/' + entry + '...'
            pull_every_n_meters(folder + '/' + entry, rate, camera)

"""
If summoned from the command line, then just go through pulling
procedure. 
"""
if __name__ == "__main__":
    if not len(sys.argv) == 4:
        print 'Usage for single file: ./pull_images_and_gps.py single [bag_file_name.bag] [rate]'
        print 'Usage for multiple files: ./pull_images_and_gps.py multiple [folder] [rate]'
    elif sys.argv[1] == 'single':
        #For now only pulls from the left camera. 
        pull_every_n_meters(sys.argv[2], sys.argv[3], 'left')
    elif sys.argv[1] == 'multiple':
        process_folder(sys.argv[2], sys.argv[3], 'left')
    else:
        print 'Usage for single file: ./pull_images_and_gps.py single [bag_file_name.bag] [rate]'
        print 'Usage for multiple files: ./pull_images_and_gps.py multiple [folder] [rate]'
