#!/usr/bin/python

"""
Takes GPS data from a bag file and 
creates a vector.txt file for a map
to be used with the cobot localization
GUI. 
"""

import rospy
import rosbag
from geopy.distance import VincentyDistance
import sys

def get_vectors_every_n_meters(gps_msgs, n):
    #Sets origin to first gps reading.  
    origin = (gps_msgs[0].latitude, gps_msgs[0].longitude)

    reading_buffer = origin

    vectors = []

    for reading in gps_msgs:
        point = (reading.latitude, reading.longitude)

        #If >= to threshold, then write vector. 
        if VincentyDistance(reading_buffer, point).meters >= n:
            #Creates vector to be written to map. 
            start_x = VincentyDistance(origin, (origin[0], reading_buffer[1])).meters
            start_y = VincentyDistance(origin, (reading_buffer[0], origin[1])).meters

            end_x = VincentyDistance(origin, (origin[0], point[1])).meters
            end_y = VincentyDistance(origin, (point[0], origin[1])).meters

            #Assigns correct sign.
            if origin[0] > reading_buffer[0]:
                start_y = -start_y

            if origin[1] > reading_buffer[1]:
                start_x = -start_x

            if origin[0] > point[0]:
                end_y = -end_y

            if origin[1] > point[1]:
                end_x = -end_x

            vectors.append([start_x, start_y, end_x, end_y])

            #Updates buffer. 
            reading_buffer = point

    return vectors

def get_gps_msgs(bag_file_name):
    bag = rosbag.Bag(bag_file_name, 'r')

    gps_msgs = []
    msg_counter = 1

    for topic, msg, t in bag.read_messages():
        if topic == '/navsat/fix':
            gps_msgs.append(msg)

            sys.stdout.write('\rRead ' + str(msg_counter) + ' messages.')
            sys.stdout.flush()
            msg_counter += 1

    #Puts cursor down one line. 
    print ''

    return gps_msgs

def get_waypoints(bag_file_name, waypoint_dist, waypoint_file_name):
    gps_msgs = get_gps_msgs(bag_file_name)
    vectors = get_vectors_every_n_meters(gps_msgs, float(waypoint_dist))

    #Writes to waypoint file. 
    waypoint_file = open(waypoint_file_name, 'w')

    for vector in vectors: 
        waypoint_file.write(str(vector[0]) + ';' + str(vector[1]) + '\n')

    waypoint_file.close()

def create_map(bag_file_name, map_file_name, origin_coordinates_file_name):
    gps_msgs = get_gps_msgs(bag_file_name)
    vectors = get_vectors_every_n_meters(gps_msgs, 0.5)

    #Writes vectors to map file. 
    map_file = open(map_file_name, 'w')

    for vector in vectors:
        vector_string = str(vector[0]) + ','
        vector_string += str(vector[1]) + ','
        vector_string += str(vector[2]) + ','
        vector_string += str(vector[3]) + '\n'

        map_file.write(vector_string)

    map_file.close()

    #Writes gps origin of map to file.
    origin_file = open(origin_coordinates_file_name, 'w')

    origin_file.write(str(gps_msgs[0].latitude) + ';' + str(gps_msgs[0].longitude))

    origin_file.close()

def print_usage():
    print 'Generate map: ./make_gps_map.py make_map [bag file] [vector out file] [origin out file]'
    print 'Get waypoints: ./make_gps_map.py get_waypoints [bag file] [waypoint distance] [waypoint out file]'

if __name__ == '__main__':
    if not len(sys.argv) == 5:
        print_usage()
    elif sys.argv[1] == 'make_map':
        create_map(sys.argv[2], sys.argv[3], sys.argv[4])
    elif sys.argv[1] == 'get_waypoints':
        get_waypoints(sys.argv[2], sys.argv[3], sys.argv[4])
    else:
        print_usage()
