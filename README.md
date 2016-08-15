This package contains an implementation of a particle filter used 
for the Jackal outdoor robot. 

Please contact Rodolfo Corona (rcorona@utexas.edu) with any questions. 

INSTALLATION
-------------

First clone the git repo into your catkin workspace:

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/rcorona/particle_filter.git

Build the package and source it:

	$ cd ~/catkin_ws
	$ catkin_make
	$ source devel/setup.bash


PARTICLE FILTER USAGE
---------------------

First start up the particle_filter_node:

	$ rosrun particle_filter particle_filter_node

If using GPS, start up the gps converter node, which
converts gps readings to map cartesian coordinates (in meters):

	$ rosrun particle_filter gps_converter [map_origin_file]

where [map_origin_file] is the txt file containing the gps
coordinates of the origin of the map you are using. 

Lastly, if you are using a bag file with the filter, start it up:

	$ rosbag play [bag_file_name]

The particle_filter_node will publish a topic called ########### TODO ##########, 
which contains a pose estimate for the robot in the form (x, y, theta) using 
a ##### TODO #### message type. 

COLLECT DATA WITH JACKAL
---------------------------

You will need: 
	* A laptop with ethernet to connect to the Jackal with while outdoors. 

	* A static IP address to use for connecting to the Jackal, go to the GETTING A STATIC IP section for some pointers
	  on how to do this. 

	* Preferably you should have some Linux distribution on the laptop for using SSH. 
	  Putty for Windows disconnects every time you physically unplug the ethernet cable, 
	  something which you'll need to do when collecting data. Linux SSH does not do
	  this, so it is more convenient. 


Before you start:
	* Make sure the Point Grey cameras are connected to the Jackal and
	  that you are using the right lense type (fisheye vs regular) for
	  your purposes.

First, connect to the Jackal via ethernet and start up 4 command line terminal windows and ssh into the jackal with each one: 
	
	$ ssh administrator@192.168.1.11 

Change the above IP address if the Jackal's IP address has been changed for some reason.  
Someone in the AMRL lab should know the password if you don't already.

Source each terminal window:

	$ source ~/.bashrc

Use two terminal windows to launch the Point Grey Camera drivers (left and right cameras):
	
	$ roslaunch pointgrey_camera_driver camera_left.launch
	$ roslaunch pointgrey_camera_driver camera_right.launch

The pointgrey camera drivers sometimes do not run properly, so you will likely have
to restart them. To check for this, check their output using the third terminal window:

	$ rostopic echo /camera_left/image_color/compressed

	and 

	$ rostopic echo /camera_right/image_color/compressed

These two topics should be publishing at roughly 10 or 12 Hz. If they are
not publishing, then restart them and check again. Repeat until 
they are both running properly. 

Now you are ready to collect data, so start collectin a ROS bag in 
the fourth terminal window:

	$ rosbag record [topics] 

If you want camera images, /camera_right/image_color/compressed
and /camera_left/image_color/compressed should be included in [topics]. 

Now, leave the SSH sessions running and physically disconnect the ethernet cable. 
You may now drive the Jackal around where you need to for data. 

NOTE: The cameras sometimes stop recording, this becomes more likely the longer
	  you leave them running. There should be a green light lit on the cameras
	  when they are recording, keep an eye on this and stop if they turn off. 

Once you are done, reconnect the ethernet cable, go to the terminal where you have
the ROS bag recording, and stop it:

	$ ^C
