#!/usr/bin/python

import sys
import matplotlib.pyplot as plt
import rospy 
import rosbag

colors = ['ro', 'bo', 'go', 'co', 'yo']

def plot_particles(rosbag_name):
    bag = rosbag.Bag(rosbag_name, 'r')
    counter = 0

    for topic, msg, t in bag.read_messages():
        if topic == '/particle_filter':
            if counter <= 30: 
                x_points = []
                y_points = []

                for i in range(len(msg.particles)):
                    x_points.append(msg.particles[i].pose.x)
                    y_points.append(msg.particles[i].pose.y)

                plt.plot(x_points, y_points, colors[counter % 5])

            counter += 1
    
    plt.gca().set_xlim([0, 50])
    plt.gca().set_ylim([-35, 30])

    plt.arrow(0, 0, 50, 0)
   
    plt.ylabel("y (m)")
    plt.xlabel("x (m)")

    #Title
    if len(sys.argv) > 2:
        title = sys.argv[2]
    else:
        title = ""

    plt.title(title)

    plt.show()
             

plot_particles(sys.argv[1])
