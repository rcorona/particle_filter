#!/usr/bin/python

"""
The functions in this script may be used in
order to crop rectangles at desired angles
from a given image. 

Author: Rodolfo Corona, rcorona@utexas.edu

Usage
-----

For cropping using pixel dimensions: ./crop_image.py pixel_dims [image] [center_x] [center_y] [angle] [width] [height] [output_image]

    Parameters:
        
        image               - The image file to crop. 

        center_x            - The x-axis coordinate pixel number for the center of the cropping. 

        center_y            - The y-axis coordinate pixel number for the center of the cropping. 

        angle               - The angle to rotate the cropped rectangle by in radians. 
        
        width               - The width in pixels of the cropped rectangle. 

        height              - The height in pixels of the cropped rectangle. 

        output_image        - The name of the image file to write the cropped rectangle to. 


For cropping using gps coordinates: ./crop_image.py gps_dims [image] [gps_corners_file] [crop_center_gps] [angle] [width] [height] [output_image]'

    Parameters:

        image               - The image file to crop. 

        gps_corner_file     - A txt file containing the gps coordinates of the corners of the
                              image that is being cropped. Should be written in the following format:

                                TOP_LEFT_LAT;TOP_LEFT_LONG
                                TOP_RIGHT_LAT;TOP_RIGHT_LONG
                                BOTTOM_LEFT_LAT;BOTTOM_LEFT_LONG
                                BOTTOM_RIGHT_LAT;BOTTOM_RIGHT_LONG

        crop_center_gps     - The gps coordinates of the center of the cropped rectangle.
                              Should be in the following format:

                                lat,long (e.g. 42.934,72.903)

        angle               - The angle in radians to rotate the cropped rectangle by. 

        width               - The width in meters of the cropped rectangle. 

        height              - The height in meters of the cropped rectangle. 

        output_image

"""

from cv2 import cv
import numpy as np
import sys
from geopy.distance import VincentyDistance

"""
This function crops an angled rectangle from
an image.

source: http://stackoverflow.com/questions/11627362/how-to-straighten-a-rotated-rectangle-area-of-an-image-using-opencv-in-python 
"""
def subimage(image, center, theta, width, height, output_image_name):
    #Creates output image and gets desired image patch. 
    output_image = cv.CreateImage((width, height), image.depth, image.nChannels)
    mapping = np.array([[np.cos(theta), -np.sin(theta), center[0]],
                                         [np.sin(theta), np.cos(theta), center[1]]])
    map_matrix_cv = cv.fromarray(mapping)
    cv.GetQuadrangleSubPix(image, output_image, map_matrix_cv)
    
    #Writes the resulting image to file. 
    cv.SaveImage(output_image_name, output_image)

def within_bounds(center_gps_coordinates, gps_corner_coordinates):
    if center_gps_coordinates[0] > gps_corner_coordinates[0][0] or center_gps_coordinates[0] < gps_corner_coordinates[2][0]:
        return False
    elif center_gps_coordinates[1] < gps_corner_coordinates[0][1] or center_gps_coordinates[1] > gps_corner_coordinates[1][1]:
        return False
    else:
        return True

"""
Crop a rotated rectangle from an image using gps coordinates as a center, 
an angle in radians, and a width and height in meters. 
"""
def crop_rectangle_with_gps(image, gps_corner_coordinates, center_gps_coordinates, angle, width, height, output_image_name):
    #Calculates pixel to meter ratio.
    ratio = float(image.width) / VincentyDistance(gps_corner_coordinates[0], gps_corner_coordinates[1]).meters
    
    #Ensures that center of cropped rectangle is within image borders. 
    if not within_bounds(center_gps_coordinates, gps_corner_coordinates):
        print 'Cropped rectangle center not within image!'

        sys.exit()

    #Calculates cropped image center using gps coordinates. 
    center_x = int(VincentyDistance(gps_corner_coordinates[0], (gps_corner_coordinates[0][0], center_gps_coordinates[1])).meters * ratio)
    center_y = int(VincentyDistance(gps_corner_coordinates[0], (center_gps_coordinates[0], gps_corner_coordinates[0][1])).meters * ratio)

    print [width, height]

    print [center_x, center_y]

    #Gets cropped image dimensions in pixels. 
    width_pixels = int(width * ratio)
    height_pixels = int(height * ratio)

    print [width_pixels, height_pixels]

    #Gets image crop using the computed information. 
    subimage(image, [center_x, center_y], angle, width_pixels, height_pixels, output_image_name)

"""
Reads a txt file containing the 
corner GPS coordinates for an image. 
"""
def read_in_gps_corners(corner_file_name):
    corner_file = open(corner_file_name, 'r')

    corners = []

    #Reads in corners in order of TL, TR, BL, BR. 
    for line in corner_file:
        if ';' in line:
            corners.append((float(line.split(';')[0]), float(line.strip().split(';')[1])))

    return corners

def extract_gps_coordinates_from_str(string):
    return (float(string.split(',')[0]), float(string.split(',')[1]))

def print_usage():
    print 'Crop using pixel dimensions: ./crop_image.py pixel_dims [image] [center_x] [center_y] [angle] [width] [height] [output_image]'
    print 'Crop using gps and radius in meters: ./crop_image.py gps_dims [image] [gps_corners_file] [crop_center_gps] [angle] [width] [height] [output_image]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'pixel_dims':
        if not len(sys.argv) == 9:
            print_usage()
        else:
            #Loads original image.    
            image = cv.LoadImage(sys.argv[2])
            
            subimage(image, (int(sys.argv[3]), int(sys.argv[4])), float(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), sys.argv[8])

    elif sys.argv[1] == 'gps_dims':
        if not len(sys.argv) == 9:
            print_usage()
        else:
            #Loads original image.
            image = cv.LoadImage(sys.argv[2])
           
            #Packages gps corner coordinates.    
            gps_corner_coordinates = read_in_gps_corners(sys.argv[3])
            center_gps_coordinates = extract_gps_coordinates_from_str(sys.argv[4])
           
            print gps_corner_coordinates

            crop_rectangle_with_gps(image, gps_corner_coordinates, center_gps_coordinates, float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), sys.argv[8])

    else: 
        print_usage()
