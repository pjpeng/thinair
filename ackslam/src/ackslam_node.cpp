//============================================================================
// Name        : mapconstruction.cpp (main file)
// Author      : Ray
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <ros/ros.h>
#include "pfslam.h"

/***************************************************
 * Lidar Callback
 * callback used to get the laserScan message and 
 * convert to parameters into the points2D struct
 *************	**************************************/

/***************************************************
 * Main
 * Main file used to handle the pf_slam including mapping
 * and receiving/handling ROS data
 *************	**************************************/
int main(int argc, char** argv) {


	ros::init(argc,argv, "ackslam");
    pfSlam ackslam; 
    ackslam.startLiveSlam();
    cout<<"start slam:"<<endl;
    //cv::Mat src(500, 500, 0);
    //unsigned char* ptr;
    //for( int i =0; i <src.rows; ++i)
    //{
    //    ptr = src.ptr<unsigned char>(i);
    //    for(int j = j = 0; j < src.cols; ++j)
    //    {
    //        ptr[j] = 255;
    //    }
    ///}
    //imshow("test",src);


    ros::spin();	
	return 0;
}
