#!/usr/bin/python

"""
The functions in this script may be used in
order to crop rectangles at desired angles
from a given image. 

Author: Rodolfo Corona, rcorona@utexas.edu
"""

from cv2 import cv
import numpy as np
import sys

"""
This function crops an angled rectangle from
an image.

source: http://stackoverflow.com/questions/11627362/how-to-straighten-a-rotated-rectangle-area-of-an-image-using-opencv-in-python 
"""
def subimage(image_name, center, theta, width, height, output_image_name):
    #Loads original image.    
    image = cv.LoadImage(image_name)
    
    #Creates output image and gets desired image patch. 
    output_image = cv.CreateImage((width, height), image.depth, image.nChannels)
    mapping = np.array([[np.cos(theta), -np.sin(theta), center[0]],
                                         [np.sin(theta), np.cos(theta), center[1]]])
    map_matrix_cv = cv.fromarray(mapping)
    cv.GetQuadrangleSubPix(image, output_image, map_matrix_cv)
    
    #Writes the resulting image to file. 
    cv.SaveImage(output_image_name, output_image)

def print_usage():
    print 'Crop using pixel dimensions: ./crop_image.py pixel_dims [image] [center_x] [center_y] [angle] [width] [height] [output_image]'

if __name__ == '__main__':
    if not len(sys.argv) >= 2:
        print_usage()

    elif sys.argv[1] == 'pixel_dims':
        if not len(sys.argv) == 9:
            print_usage()
        else:
            subimage(sys.argv[2], (int(sys.argv[3]), int(sys.argv[4])), float(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]), sys.argv[8])

    else: 
        print_usage()
