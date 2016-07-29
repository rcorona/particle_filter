#!/usr/bin/python 

"""
This script is used to visualize output from 
the particle_filter node in the particle_filter
package.

Author: Rodolfo Corona, rcorona@utexas.edu
"""

import turtle
import rospy
from particle_filter.msg import Particle_vector

"""
This class implements the visualization for 
the particle filter. 
"""
class ParticleVisualizer:

    """
    Initializes the display and subscribes
    to particle_filter topic. 
    """
    def __init__(self):
        #Creates display for visualization. 
        self.window = turtle.Screen()
        turtle.setworldcoordinates(-5, -5, 5, 5)
        turtle.speed(0)

        #Initializes ROS and subscribes to particle filter. 
        rospy.init_node('particle_visualizer', anonymous=True)
        rospy.Subscriber('particle_filter', Particle_vector, self.update)

        #Initializes list of particles. 
        self.particles = []

    """
    Initializes particle number to match the
    number found the in the Particle_vector message
    and creates all the turtle objects to display. 
    """
    def init_particles(self, particle_vector): 
        #Clears particles. 
        self.particles = []

        #Creates a turtle object for each particle. 
        for i in range(len(particle_vector)):
            #Creates turtle and appends it. 
            new_turtle = turtle.Turtle()
            new_turtle.penup()
            new_turtle.color('red')

            self.particles.append(new_turtle)

    """
    Updates the positions of each particle using
    the given readings and re-draws them on the display. 
    """
    def update_particles(self, particle_vector):
        #Updates each particle given the new reading. 
        for i in range(len(particle_vector)):
            #Gets readings. 
            x = particle_vector[i].pose.x
            y = particle_vector[i].pose.y
            theta = particle_vector[i].pose.theta

            #Updates values based on readings.
            self.particles[i].hideturtle()
            self.particles[i].setx(x)
            self.particles[i].sety(y)
            self.particles[i].showturtle()


    """
    The callback method for when particle_filter messages
    are received. This will initialize the particles if 
    necessary as well as set their positions accordingly. 
    """
    def update(self, particle_vector):
        #Re-initialize particles if number has changed. 
        if len(self.particles) != len(particle_vector.particles):
            self.init_particles(particle_vector.particles)

        #Updates particle positions. 
        self.update_particles(particle_vector.particles)

    """
    Runs the visualizer. Currently only
    calls the looping method for the 
    display. 
    """
    def run(self):
        turtle.mainloop()

"""
Instantiate a visualizer and run it. 
"""
if __name__ == "__main__":
    pv = ParticleVisualizer()
    pv.run()
