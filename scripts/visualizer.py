#!/usr/bin/python

"""
Used to visualize different types of Jackal data
from bag files in a variety of ways. 
"""

import rospy
import rosbag
import sys
from math import sqrt
import math
import os
from overlay_to_map import read_in_overlay_specs 
import matplotlib.pyplot as plt
from geopy.distance import VincentyDistance
from scipy.misc import imread
import tf
import numpy as np

class Visualizer: 
    def __init__(self, bag_file, map_specs):
        #Opens bag file. 
        print 'Opening bag file...'
        self.bag_file = rosbag.Bag(bag_file, 'r')

        #Determines coordinates of corner of map image if overlay given.
        if map_specs == 'None':
            self.map_img_file = None
        else:
            map_img_name, self.BR, self.TL = read_in_overlay_specs(map_specs)
            self.width = float(VincentyDistance((self.BR[0], self.TL[1]), (self.BR[0], self.BR[1])).meters)
            self.height = float(VincentyDistance((self.BR[0], self.TL[1]), (self.TL[0], self.TL[1])).meters)

            #Loads map image onto plot using corner coordinates. 
            self.figure = plt.figure()
            self.map_img_file = imread(map_img_name)

    def gps_to_meters(self, latitude, longitude):
        #Translates coordinate point based on formula below. 
        h = float(VincentyDistance((self.BR[0], self.TL[1]), (latitude, longitude)).meters)
        e = float(VincentyDistance((self.BR[0], self.BR[1]), (latitude, longitude)).meters)

        x = ((h ** 2) + (self.width ** 2) - (e ** 2)) / (2 * self.width)
        y = sqrt((h ** 2) - (x ** 2))

        return [x, y]

    def get_heading_from_gps_points(self, x_points, y_points):
        print 'Calculating heading...'

        #Need at least two points to calculate heading. 
        if (len(x_points) < 2 or len(y_points) < 2):
            print "Need at least 2 points to compute heading!!!" 

            sys.exit()
    
        #Computes heading for each point in gps path. 
        heading_points = []

        index = 0

        while index < len(x_points):
            #Corner cases. 
            if index == 0:
                v1 = np.array([x_points[0], y_points[0]])
                v2 = np.array([x_points[1], y_points[1]])

                #Gets unit vector with heading. 
                heading_vector = v2 - v1
                heading_vector /= np.linalg.norm(heading_vector)

                #Computes heading. 
                heading_points.append(math.atan2(heading_vector[1], heading_vector[0]))
            elif index == len(x_points) - 1:
                v1 = np.array([x_points[index - 1], y_points[index - 1]])
                v2 = np.array([x_points[index], y_points[index]])

                #Gets unit vector with heading. 
                heading_vector = v2 - v1
                heading_vector /= np.linalg.norm(heading_vector)

                #Computes heading. 
                heading_points.append(math.atan2(heading_vector[1], heading_vector[0]))
            #Have 3 points to better approximate heading. 
            else:
                v1 = np.array([x_points[index - 1], y_points[index - 1]])
                v2 = np.array([x_points[index], y_points[index]])
                v3 = np.array([x_points[index + 1], y_points[index + 1]])

                #Gets translational vectors between points. 
                diff1 = v2 -  v1
                diff2 = v3 - v2
                
                #Approximates heading by getting half of vector for full translation. 
                V = (diff1 + diff2) / 2
                V /= np.linalg.norm(V)

                #Approximates heading. 
                heading_points.append(math.atan2(V[1], V[0]))
                
            #Updates progress. 
            index += 1

            sys.stdout.write('\rCalculated ' + str(index) + ' orientations...')
            sys.stdout.flush()

        #Last point technically has no heading, so assign previous value to it. 
        heading_points.append(heading_points[len(heading_points) - 1])

        return heading_points


    def plot_gps(self, topic_data):
        print 'Plotting gps...'
            
        x_points = []
        y_points = []

        #Processes each message.  
        for msg_time in topic_data['/navsat/fix']:
            msg = msg_time[0]
    
            x, y = self.gps_to_meters(msg.latitude, msg.longitude)

            #Appends gps reading only if it differs from last one. 
            if len(x_points) == 0 or not (x == x_points[len(x_points) - 1] and y == y_points[len(y_points) - 1]):
                x_points.append(x)
                y_points.append(y)

        #Gets heading estimates from GPS. 
        theta_points = self.get_heading_from_gps_points(x_points, y_points)
        
        #Plots heading. 
        plt.plot(range(len(theta_points)), theta_points)
        plt.show()

        #Places map overlay on graph if needed. 
        if not self.map_img_file == None:
            plt.imshow(self.map_img_file, zorder=0, extent=[0.0, self.width, 0.0, self.height])

        #Plots arrows. 
        for i in range(len(x_points)): 
            x = x_points[i]
            y = y_points[i]
            theta = theta_points[i]

            length = 1
            x_len = math.cos(theta)
            y_len = math.sin(theta)

            plt.arrow(x, y, x_len, y_len, fc = 'k', ec = 'k', head_width = 0.1, head_length = 0.1)

        plt.show()

        #Plots points. 
        plt.plot(x_points, y_points, 'ro')
        plt.show()

    def plot_odom_orientation(self, topic_data):
        theta_points = []
        time_points = []

        for msg_time in topic_data['/odometry/filtered']:
            #Gets message and time from data. 
            msg = msg_time[0]
            time = msg_time[1].to_sec()

            #Gets quaternion from orientation readings.
            x = msg.pose.pose.orientation.x
            y = msg.pose.pose.orientation.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w

            #Converts it to Euler angle. 
            euler = tf.transformations.euler_from_quaternion((x,y,z,w))

            #Euler angles. 
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            #Only need yaw for now. 
            theta_points.append(yaw)
            
            #Includes time stamp for use in plotting. 
            time_points.append(time)

        #Plots points. 
        plt.plot(time_points, theta_points, 'ro')
        plt.show()       

    def plot_odom(self, topic_data):
        x_points = []
        y_points = []

        #Processes each message. 
        for msg_time in topic_data['/odometry/filtered']:
            msg = msg_time[0]    

            x_points.append(msg.pose.pose.position.x)
            y_points.append(msg.pose.pose.position.y)
        
        #Plots points. 
        plt.plot(x_points, y_points, 'ro')
        plt.show()

    def plot_odom_overlay(self, topic_data):
        x_points = []
        y_points = []

        #Gets difference in order to synchronize coordinate planes.
        start_gps = topic_data['/navsat/fix'][0]
        x_0, y_0 = self.gps_to_meters(start_gps.latitude, start_gps.longitude)

        start_odom = topic_data['/odometry/filtered'][0]
        dx = x_0 - start_odom.pose.pose.position.x
        dy = y_0 - start_odom.pose.pose.position.y

        #Processes each message. 
        for msg_time in topic_data['/odometry/filtered']:
            msg = msg_time[0]    

            x = msg.pose.pose.position.x + dx
            y = msg.pose.pose.position.y + dy

            x_points.append(x)
            y_points.append(y)

        #Plots points. 
        plt.plot(x_points, y_points, 'ro')
        plt.show()

    def plot_imu(self, topic_data): 
        theta_points = []
        time_points = []

        for msg_time in topic_data['/imu/data']:
            #Gets message and time from data. 
            msg = msg_time[0]
            time = msg_time[1].to_sec()

            #Gets quaternion from orientation readings.
            x = msg.orientation.x
            y = msg.orientation.y
            z = msg.orientation.z
            w = msg.orientation.w

            #Converts it to Euler angle. 
            euler = tf.transformations.euler_from_quaternion((x,y,z,w))

            #Euler angles. 
            roll = euler[0]
            pitch = euler[1]
            yaw = euler[2]

            #Only need yaw for now. 
            theta_points.append(yaw)
            
            #Includes time stamp for use in plotting. 
            time_points.append(time)

        #Plots points. 
        plt.plot(time_points, theta_points, 'ro')
        plt.show()       

    def plot_data(self, topic, topic_data):
        if topic == '/navsat/fix':
            self.plot_gps(topic_data)
        elif topic == '/odometry/filtered':
            self.plot_odom(topic_data)
        elif topic == '/imu/data':
            self.plot_imu(topic_data)
        elif topic == '/odometry/filtered_overlay':
            self.plot_odom_overlay(topic_data)
        elif topic == '/odometry/filtered_pos':
            self.plot_odom_orientation(topic_data)

    def plot_topic(self, topic_name):
        #Compiles all data for relevant topic. 
        topic_data = {'/navsat/fix': [], 
                      '/odometry/filtered': [],
                      '/imu/data': []}

        #Keeps track of reading progress. 
        msg_count = 0
        total_msgs = self.bag_file.get_message_count()

        for topic, msg, t in self.bag_file.read_messages():
            #Outputs progress. 
            sys.stdout.write('\rRead ' + str(msg_count) + ' msgs out of ' + str(total_msgs) + '...')
            sys.stdout.flush()
            msg_count += 1        

            #Checks if msg topic is to be visualized.
            if topic in topic_data:
                topic_data[topic].append([msg, t])
   
        #Prints new line for formatting. 
        print ''

        #Plots data. 
        self.plot_data(topic_name, topic_data)

if __name__ == '__main__':
    visualizer = Visualizer(sys.argv[1], sys.argv[2])
    
    visualizer.plot_topic(sys.argv[3])
            
