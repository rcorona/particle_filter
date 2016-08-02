/*
    This program is used to undistort an image. 

    Author Sourish Ghosh

    Usage: ./undistort [camera yml file] [distorted_img] [undistorted_img]

	Parameters:
		[camera yml file]: The .yml file which contains specifications needed
						   for undistortion for the camera used to take the
						   undistorted image. This will either be cam_left.yml
						   or cam_right.yml for the left and right cameras 
						   respectively. 

 		[distorted_img]:   Path to the image you would like to undistort. 

		[undistorted_img]: Path to the undistorted image you would like to create. 
 */

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char const *argv[])
{
  if (argc != 4) {
  	cout << "Usage: ./undistort [camera yml file] [distorted_img] [undistorted_img]" << endl; 

	return 0; 
  }

  cv::Mat K1, K2;
  cv::Vec4d D1, D2;

  //Loads distorted image. 
  cv::Mat img = imread(argv[2], CV_LOAD_IMAGE_COLOR);

  //Resizes before undistorting. 
  cv::Mat img_resized;  
  resize(img, img_resized, Size(960, 600));

  //Loads camera specifications from the yml file. 
  cv::FileStorage fs1(argv[1], cv::FileStorage::READ);
  fs1["K1"] >> K1;
  fs1["D1"] >> D1; 
  Matx33d K1new = Matx33d(K1); 

  //Undistorts images.
  cv::Mat u1;  
  cv::fisheye::undistortImage(img_resized, u1, Matx33d(K1), Mat(D1), K1new);

  //Resizes the images to their final resolution. 
  cv::Mat u2; 
  resize(u1, u2, Size(360, 224));

  //Writes final image to file. 
  cout << "writing to file..." << endl;
  imwrite(argv[3], u2);
  
  return 0;
}
