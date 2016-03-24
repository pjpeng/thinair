/*
 *
 * pfslam.h
 *
 *  Created on: Aug 10, 2015
 *      Author: robot
 */

#ifndef PFSLAM_H_
#define PFSLAM_H_

#include <ackslam/Notation.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/GetMap.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

using namespace cv;



class pfSlam
{
//  public functions
public:
    pfSlam();


	virtual ~pfSlam();  //

	void init();
	float NormalDistribute();
	pose ResetPartical(double x, double y,double theta);
	void Positioning_by_PF(points2D* frame_ptr);
	void Positioning_by_PF_Odometer(points2D* frame_ptr);


	//----localization based map recontruction
	void MappingPreProc();
	void MappingUpdate(points2D* frame_ptr);
	void MappingPostProc(points2D* frame_ptr);

	void Mapping_StaticMapBuilding(points2D* frame_ptr);

    
    //---added by zy for ros wrapper
    //  main loop of slam 
    void startLiveSlam();
    void publishTransform();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr&
            scan);
    bool mapCallback(nav_msgs::GetMap::Request &req,
            nav_msgs::GetMap::Response &res);
    void publishLoop(double transform_publish_period);
    


//	public variables
public:

    //for debugging display 
    Mat display_src;
    Mat test_src;
    //
    //
    //
    //
    points2D frame; 
	int forward_laser_index;
	float globalmapresolution;
	int slamogmsize;

	SVehicleStateToNet m_sVehicleStateToNet;

	double *prob_global, *tmp_prob_global, *prob_global_static,*tmp_prob_global_static;

	double *prob_global_frame;  // scan data displayed in the map of slam
	double *prob_local;  //local map data from global map

	// mapping based particle filter localization
	bool filter_initialized;
	bool localization_initialized;

	state_struct INSpose_pre, INSpose_cur;
	pos slammap_center_pos;
	pose vpose[SAMPLENO];
	pose noisepos[SAMPLENO];

	float raytracing_angleresolution;//equal to gsp_laser_angle_increment_?
	int raytracing_angle_count;
	int raytracing_lmap_count;
	int* raytracing_x_array;
	int* raytracing_y_array;

	float x_minrange, x_maxrange, y_minrange, y_maxrange;
	float std_err;

	//key parameters
	float particle_position_boundary;
	float particle_theta_boundary;
	float move_theta_std ;
	float move_std;
	float particle_position_std_threld;
	float measurement_convriance;
	float sickangle_err_std;
	float sickrange_err_std;
	float sickangle_err_std2;
	float sickrange_err_std2;
    //-- added by Yu Zhang for ROS wrapper 
    //-- for display 
    
    
private:
    ros::NodeHandle node_;
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;
    tf::TransformListener tf_;
    message_filters::Subscriber<sensor_msgs::LaserScan>*scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
    tf::TransformBroadcaster* tfB_;

    bool got_map_;
    nav_msgs::GetMap::Response map_;
    
    ros::Duration map_update_interval_;
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::mutex map_mutex_;

    boost::thread* transform_thread_;
    std::string base_frame_;
    std::string laser_frame_;
    std::string map_frame_;
    std::string odom_frame_;

    
    
    ros::NodeHandle private_nh_;
    unsigned long int seed_;
    
    double transform_publish_period_;
    double tf_delay_;
    
};



#endif /* PFSLAM_H_ */
