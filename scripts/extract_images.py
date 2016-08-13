#!/usr/bin/python

"""
This script extracts all images from a topic
within a bag file. 
"""

import rospy
import rosbag
import sys

def extract_images(bag_file_name, topic_name, image_file_folder):
    #Opens given bag file. 
    bag = rosbag.Bag(bag_file_name, 'r')

    #Images are named based on their sequential order. 
    img_counter = 0

    #Used to keep track of progress. 
    msg_counter = 0
    num_msgs = bag.get_message_count() 

    #Writes each image to its own jpg file. 
    for topic, msg, t in bag.read_messages():
        if topic == topic_name:
            image_file = open(image_file_folder + '/' + str(img_counter) + '.jpg', 'w')
            image_file.write(msg.data)
            image_file.close()

            img_counter += 1

        msg_counter += 1

        #Writes progress. 
        sys.stdout.write('\rRead through ' + str(msg_counter) + ' out of ' + str(num_msgs) + ' messages.')
        sys.stdout.flush()

    #Puts terminal cursor on next line. 
    print ''

if __name__ == '__main__':
    extract_images(sys.argv[1], sys.argv[2], sys.argv[3])
