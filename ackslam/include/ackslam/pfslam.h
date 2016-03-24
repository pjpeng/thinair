/*
 * pfslam.h
 *
 *  Created on: Aug 10, 2015
 *      Author: robot
 */

#ifndef PFSLAM_H_
#define PFSLAM_H_

#include "Notation.h"

class pfSlam
{
//  public functions
public:



	virtual ~pfSlam();  //

	void init();
	float NormalDistribute();
	pose ResetPartical(float x, float y,float theta);
	void Positioning_by_PF(points2D* frame_ptr);
	void Positioning_by_PF_Odometer(points2D* frame_ptr);


	//----localization based map recontruction
	void MappingPreProc();
	void MappingUpdate(points2D* frame_ptr);
	void MappingPostProc(points2D* frame_ptr);

	void Mapping_StaticMapBuilding(points2D* frame_ptr);

//	public variables
public:

	int forward_laser_index;
	float globalmapresolution;
	int slamogmsize;

	SVehicleStateToNet m_sVehicleStateToNet;

	double *prob_global, *tmp_prob_global, *prob_global_static,*tmp_prob_global_static;
	double *prob_global_frame;  // scan data displayed in the map of slam
	double *prob_local;  //local map data from global map

	// mapping based particle filter localization
	bool filter_initialized;
	bool localization_initilized;

	state_struct INSpose_pre, INSpose_cur;
	pos slammap_center_pos;
	pose vpose[SAMPLENO];
	pose noisepos[SAMPLENO];

	float raytracing_angleresolution;
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
};



#endif /* PFSLAM_H_ */
