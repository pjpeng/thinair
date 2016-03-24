/*
 * pfslam.cpp
 *
 *  Created on: Aug 18, 2015
 *      Author: Yu Zhang
 */

#include "pfslam.h"
#include <ackslam/random/random.h>:
#include <ackslam/Basicfunc.h>
#include <iostream> 
#include <cfloat>
#include "ros/ros.h"
#include "ros/console.h"
#include <boost/foreach.hpp> 

#define MAXPROB           0.9f//12.0
#define MINPROB           0.3f//2.0
#define UNKOWNPROB        0.5f//4.0
#define EPSILON_ERROR     0.000001f

using namespace cv;
pfSlam::pfSlam():
    map_to_odom_(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Point(0,0,0))),
    private_nh_("~"), scan_filter_sub_(NULL), scan_filter_(NULL), transform_thread_(NULL)
{
    seed_ = time(NULL);



    init();
}

pfSlam::~pfSlam()
{
    if(transform_thread_)
    {
        transform_thread_->join();
        delete transform_thread_;
    }
    if(scan_filter_)
        delete scan_filter_;
    if(scan_filter_sub_)
        delete scan_filter_sub_;
    
	delete[] prob_global;
	delete[] tmp_prob_global;

	delete[] prob_global_static;
	delete[] tmp_prob_global_static;

	delete[] prob_global_frame;

	delete[] prob_local;

	delete[] raytracing_x_array;
	delete[] raytracing_y_array;
}

void pfSlam::init()
{
    ////////////////////////////////////////////////////////////////////////
    ///////////////////   ROS code 
    tfB_ = new tf::TransformBroadcaster();
    ROS_ASSERT(tfB_);
    got_map_ = false;
    
    if (!private_nh_.getParam("base_frame",base_frame_))
        base_frame_ = "base_point";
    if (!private_nh_.getParam("map_frame", map_frame_))
        map_frame_ = "map" ;
    if (!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    private_nh_.param("transform_publish_period",transform_publish_period_, 0.05);
    laser_frame_ = "fastHokuyo";

    double tmp;
    if(!private_nh_.getParam("map_update_interval",tmp))
        tmp = 5.0;
    map_update_interval_.fromSec(tmp);

    if(!private_nh_.getParam("tf_delay",tf_delay_))
        tf_delay_ = transform_publish_period_;

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

    localization_initialized = false;
    filter_initialized = false;
    forward_laser_index=cvRound((SICKPTNUM-1)*SICKANGLERESOLUTION)/2;

    // slammap center initialize 
    slammap_center_pos.x = 0 ;
    slammap_center_pos.y = 0 ;


    // set the size of the ogm : height 100  width 100
    double slamogmsize_m=100.0f;
	globalmapresolution = OGMRESOLUTION ;
	slamogmsize=cvRound(slamogmsize_m / globalmapresolution+1);

    // output the size  resolution of the map 
    cout <<" slamogmsize:"<< slamogmsize<<endl;
    cout <<" globalmapresolusion:"<<globalmapresolution<<endl;
    

	prob_global=new double[slamogmsize*slamogmsize];
	memset(prob_global,0,sizeof(double)*slamogmsize*slamogmsize);
	tmp_prob_global=new double[slamogmsize*slamogmsize];
	memset(tmp_prob_global,0,sizeof(double)*slamogmsize*slamogmsize);

    prob_global_static = new double[slamogmsize*slamogmsize];
    memset(prob_global_static, 0 , sizeof(double)*slamogmsize*slamogmsize);
    tmp_prob_global_static= new double[slamogmsize*slamogmsize];
    memset(tmp_prob_global_static, 0, sizeof(double)*slamogmsize*slamogmsize);



    ///////////////////////////////////////////////////
    // for display the result of map
    cv::Mat src(slamogmsize,slamogmsize, 0);

    //display_src=src;
    uchar* ptr;
    for(int i = 0; i <src.rows; ++i)
    {
        ptr=src.ptr<uchar>(i);
        for(int j= 0; j < src.cols; ++j)
        {
            ptr[j] = 255;
        }
    }
    namedWindow("test",WINDOW_AUTOSIZE);

    cvtColor(src,display_src, CV_GRAY2RGB);
    ///////////////////////////////////////////////////

    //namedWindow("stest",WINDOW_AUTOSIZE);

    //test_src = display_src.clone();


    //circle(test_src,Point(10,10),10,Scalar(255,0,0),-1,8,0);
    
    //circle(test_src,Point(450,450),4,Scalar(0,255,0),-1,8,0);
    //circle(test_src,Point(450,10),4,Scalar(0,0,255),-1,8,0);
    //circle(test_src,Point(250,250),10,Scalar(255,0,0),-1,8,0);
    //int x = cvRound(250+5*cos(pi/4));
    //int y = cvRound(250+5*sin(pi/4));
    //cout<<"test point:"<<x<<"  "<<y<<endl;

    //circle(test_src,Point(x,y),10,Scalar(0,0,0),-1,8,0);
    
    //imshow("stest",test_src);
    //waitKey(10000);

    
	prob_global_frame=new double[slamogmsize*slamogmsize];
	memset(prob_global_frame,0,sizeof(double)*slamogmsize*slamogmsize);
    

	prob_local=new double[OGMWIDTH_CELL*OGMHEIGHT_CELL];
	memset(prob_local,0,sizeof(double)*OGMWIDTH_CELL*OGMHEIGHT_CELL);

	raytracing_angleresolution = 20.0f;
	raytracing_angle_count = cvRound(360.0f / raytracing_angleresolution);
	raytracing_lmap_count = cvRound(70.0f / globalmapresolution);
	raytracing_x_array = new int[raytracing_angle_count * raytracing_lmap_count];
	raytracing_y_array = new int[raytracing_angle_count * raytracing_lmap_count];

	for(int i = 0 ; i < raytracing_angle_count; i++){
			double raytracing_angle = raytracing_angleresolution * i;
			double raytracing_radius = raytracing_angle * pi / 180;

			for(int j = 0 ; j < raytracing_lmap_count; j++){
				double lmap = j * globalmapresolution;
				raytracing_x_array[i * raytracing_lmap_count + j] = cvRound(lmap * cos(raytracing_radius) / globalmapresolution);
				raytracing_y_array[i * raytracing_lmap_count + j] = cvRound(lmap * sin(raytracing_radius) / globalmapresolution);
			}
	}

	particle_position_boundary = 10.0f;

	particle_theta_boundary =  15.0f * pi / 180.0f;

	move_std = 5.0f;

	move_theta_std = 10.0f * pi / 180.0f;

	measurement_convriance = 0.6f;

	particle_position_std_threld = 1.0f;

	sickangle_err_std = 2.0f;
	sickrange_err_std = 1.0f;
	sickangle_err_std2 = 1.0f;
	sickrange_err_std2 = 0.2f;

	m_sVehicleStateToNet.fFLRWheelAverAngle=0;
	m_sVehicleStateToNet.fForwardVel=0;
	m_sVehicleStateToNet.fDeForwardVel=0;
	m_sVehicleStateToNet.fPos_east=0;
	m_sVehicleStateToNet.fPos_north=0;
	m_sVehicleStateToNet.fDRPos_E=0;
	m_sVehicleStateToNet.fDRPos_N=0;
	m_sVehicleStateToNet.fHeading=0;
	m_sVehicleStateToNet.fTheta=0;
	m_sVehicleStateToNet.f_shift=4;
	m_sVehicleStateToNet.f_shift1=0;
	m_sVehicleStateToNet.f_leftlamp=0;
	m_sVehicleStateToNet.f_rightlamp=0;
	m_sVehicleStateToNet.f_estop=0;
	m_sVehicleStateToNet.lateralctrl_enabled=0;
	m_sVehicleStateToNet.longitutdectrl_enabled=0;
	m_sVehicleStateToNet.brake_enabled = 0;
	m_sVehicleStateToNet.lTimeStamp = 0 ;
	m_sVehicleStateToNet.fold_Pos_east=0;
	m_sVehicleStateToNet.fold_Pos_north=0;
	m_sVehicleStateToNet.fold_DRPos_E=0;
	m_sVehicleStateToNet.fold_DRPos_N=0;
	m_sVehicleStateToNet.f_integrated_east=0;
	m_sVehicleStateToNet.f_integrated_north=0;
	m_sVehicleStateToNet.f_slamcorrected_east=0;
	m_sVehicleStateToNet.f_slamcorrected_north=0;
	m_sVehicleStateToNet.f_slamcorrected_theta = 0;
	m_sVehicleStateToNet.FONSValid = true;
	m_sVehicleStateToNet.Odometer_theta = 0.0f;
	m_sVehicleStateToNet.Odometer_x = 0.0f;
	m_sVehicleStateToNet.Odometer_y = 0.0f;
	m_sVehicleStateToNet.Odometer_pre = 0.0f;

    for(int i= 0; i<761; i++)
    {
        frame.distance[i] = 0;
    }


		/*lTime.QuadPart = 100000;
		lTimeSpan.QuadPart = 100000;

		mapreconstructionsharedmemorysize1 = sizeof(LARGE_INTEGER) + sizeof(points2D) + sizeof(SVehicleStateToNet);
		Buf1 = new char[mapreconstructionsharedmemorysize1];

		mapreconstructionsharedmemorysize2 = sizeof(LARGE_INTEGER) + sizeof(double)*slamogmsize*slamogmsize + sizeof(double)*OGMHEIGHT_CELL*OGMWIDTH_CELL +
			sizeof(pos) + sizeof(SVehicleStateToNet);
		Buf2 = new char[mapreconstructionsharedmemorysize2];

		MapReconstruction_mutex1 = CreateMutex(NULL,TRUE,"Map Client Reconstruction Mutex1");
		ReleaseMutex(MapReconstruction_mutex1);

		MapReconstruction_mutex2 = CreateMutex(NULL,TRUE,"Map Client Reconstruction Mutex2");
		ReleaseMutex(MapReconstruction_mutex2);*/


}

float pfSlam::NormalDistribute(void)
{
	static int iset = 0;
		static double gset;
		double fac, rsq, v1, v2;
		if (iset == 0)
		{
			do
			{
				MSLRandomSource R;
				double temprand;
				R >> temprand;
				v1 = 2 * temprand - 1;
				R >> temprand;
				v2 = 2 * temprand - 1;
				rsq = v1 * v1 + v2 * v2;
			} while (rsq >= 1.0 || rsq == 0.0);

			fac = (double)sqrt(-2.0 * log(rsq) / rsq);
			gset = v1 * fac;
			iset = 1;
			return (double)(v2 * fac);
		}
		else
		{
			iset = 0;
			return (double)gset;
		}
}

pose pfSlam::ResetPartical(double x, double y,double theta)
{
	pose particle;

	MSLRandomSource R;
	double temprand;

    //cout<<"x y :"<<x<<" "<<y<<endl;

	R >> temprand;
    //cout<<"temprand:"<<temprand<<endl;
	particle.x = x + particle_position_boundary * 2 * ((double)temprand - 0.5f);/*NormalDistribute();*/
    //cout<<"particle_position_boundary:"<<particle_position_boundary<<endl;
    //cout<<"temprand:"<<(double)temprand<<endl;
    //cout<<" in reset x process:"<<particle.x<<endl;
    
	R >> temprand;
	particle.y = y + particle_position_boundary * 2 * ((double)temprand - 0.5f);/*NormalDistribute();*/
	//cout<<" in reset y process:"<<particle.y<<endl;
    R >> temprand;
	particle.theta=theta + particle_theta_boundary * 2 * ((double)temprand - 0.5f);/*NormalDistribute();*/
	if(particle.theta > pi * 2) particle.theta = particle.theta - pi * 2;
	else if(particle.theta <= 0) particle.theta = particle.theta + pi * 2;

	particle.weight = 1.0 / SAMPLENO;
    //cout<<"reset particle :"<<particle.x <<" "<<particle.y<<endl;

	return particle;
}

void pfSlam::Positioning_by_PF(points2D *frame_ptr)
{
    // feed the DR pos to INSpose  
	    //cout<<"Positioning by pf is running"<<endl;
        INSpose_cur.position.x = m_sVehicleStateToNet.fDRPos_E ;
		INSpose_cur.position.y = m_sVehicleStateToNet.fDRPos_N ;
		INSpose_cur.theta = m_sVehicleStateToNet.fTheta;


		double delta_x = (INSpose_cur.position.x - INSpose_pre.position.x) ;
		double delta_y = (INSpose_cur.position.y - INSpose_pre.position.y) ;
		double delta_s = sqrt(delta_x * delta_x + delta_y * delta_y);

		double delta_theta = INSpose_cur.theta - INSpose_pre.theta;

		if(delta_theta >= pi) delta_theta = delta_theta - pi * 2;
		else if(delta_theta < -pi) delta_theta = delta_theta + pi * 2;


		INSpose_pre.position.x=INSpose_cur.position.x;
		INSpose_pre.position.y=INSpose_cur.position.y;
		INSpose_pre.theta=INSpose_cur.theta;


		//if(fabs(delta_x) > EPSILON_ERROR || fabs(delta_y) > EPSILON_ERROR || fabs(delta_theta) > EPSILON_ERROR)
		{
            

            
			m_sVehicleStateToNet.f_slamcorrected_east = m_sVehicleStateToNet.f_slamcorrected_east + delta_x;
			m_sVehicleStateToNet.f_slamcorrected_north = m_sVehicleStateToNet.f_slamcorrected_north + delta_y;
		    m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.f_slamcorrected_theta + delta_theta;


			// initial slam
			double maxweight = 0.0;
			x_minrange = INVALIDVALUE;
			x_maxrange = -INVALIDVALUE;
			y_minrange = INVALIDVALUE;
			y_maxrange = - INVALIDVALUE;

			if(1){

				MSLRandomSource R;
				double temprand;

				// localization
				if(!filter_initialized){


					//initial the particles
					for(int j=0;j<SAMPLENO;j++){
						noisepos[j] = ResetPartical(m_sVehicleStateToNet.f_slamcorrected_east,
							m_sVehicleStateToNet.f_slamcorrected_north, m_sVehicleStateToNet.f_slamcorrected_theta);
                    }


					filter_initialized = true;
				}
				else{
					//// set the result of resampling to noisepos ,introduce 10% new particles 
					for(int j = 0 ;j< cvRound(SAMPLENO * 0.9); j++){
						noisepos[j].theta = vpose[j].theta;
						noisepos[j].x = vpose[j].x;
						noisepos[j].y = vpose[j].y;
						noisepos[j].weight = vpose[j].weight;
					}
					for(int j =cvRound(SAMPLENO * 0.9); j < SAMPLENO ; j++){
						noisepos[j] = ResetPartical(m_sVehicleStateToNet.f_slamcorrected_east,
							m_sVehicleStateToNet.f_slamcorrected_north, m_sVehicleStateToNet.f_slamcorrected_theta);
					}

					// motion update 
					{
						for(int j = 0 ; j< SAMPLENO; j++){
							//R >> temprand;
                            
							noisepos[j].theta = noisepos[j].theta + delta_theta +  move_theta_std * NormalDistribute();/*2 * (temprand - 0.5)*/
							while(noisepos[j].theta >= 2 * pi)
								noisepos[j].theta = noisepos[j].theta - 2 * pi;
							while(noisepos[j].theta < 0)
							    noisepos[j].theta = noisepos[j].theta + 2 * pi;
                            
							double costheta = cos(noisepos[j].theta);
							double sintheta = sin(noisepos[j].theta);
							double delta_s_sample = move_std * NormalDistribute();
							noisepos[j].x = noisepos[j].x + (delta_s + delta_s_sample) * costheta;
							noisepos[j].y = noisepos[j].y + (delta_s + delta_s_sample) * sintheta;
						}
					}

				}
                //output the noise pose 
                

				// reset the weight of particles according to the mapping results 
				{
					for(int j = 0 ; j< SAMPLENO; j++){
						// particles are not on the obstacles should be set to 60m  
						double p = 1.0;

						pos sensorpos_global ;
						sensorpos_global.x = noisepos[j].x + cos(noisepos[j].theta) * VEHICLELEN_M;
						sensorpos_global.y = noisepos[j].y + sin(noisepos[j].theta) * VEHICLELEN_M;

                        //cout<<"sensorpos_global:"<<sensorpos_global.x <<" "<<sensorpos_global.y<<endl;
						{
							for(int n=0; n < raytracing_angle_count; n++){
								double serangle = n * raytracing_angleresolution;

								double err = serangle - (noisepos[j].theta) * 180 / pi;
								while(err >= 360)
									err = err-360;

								while(err < 0)
									err = err+360;

								if(err <= 95 || err >= 265){

									//calculate the index of the laser 
                                    unsigned int index=0;
									if(err <= 95){
										index = cvRound(err / SICKANGLERESOLUTION) + 381;
									}
									else{
										index=cvRound((err - 360) / SICKANGLERESOLUTION) + 381;
									}

                                    //get the distance 
									double len=frame_ptr->distance[index];

									{
										int x,y;
										Convert_World_to_OGM(sensorpos_global.x - slammap_center_pos.x, sensorpos_global.y - slammap_center_pos.y, &x, &y,
											(slamogmsize - 1) / 2 * globalmapresolution, (slamogmsize - 1) / 2 * globalmapresolution, globalmapresolution);

										//get the sensorpos in gridmap in cell. pass it to x,y 
                                        double bestlmap = 1000.0, tmp_mindis =1000.0;

										bool obstacle_detected = false;

										int* raytracing_x_array_startptr = &raytracing_x_array[n * raytracing_lmap_count];
										int* raytracing_y_array_startptr = &raytracing_y_array[n * raytracing_lmap_count];
										for(int i = 0 ; i < raytracing_lmap_count; i++){
											int deltax = *(raytracing_x_array_startptr + i);
											int deltay = *(raytracing_y_array_startptr + i);
											int x1 = x + deltax;
											int y1 = y + deltay;

											double lmap = i * globalmapresolution;

											if(x1>=0 && x1<slamogmsize && y1>=0 && y1<slamogmsize){

												if(prob_global[x1 + y1*slamogmsize]  > MAXPROB - EPSILON_ERROR){
													obstacle_detected = true;
													double tmpdis = fabs(lmap - len);
													//search for the best matched length with len, and record the best matched length bestlmap
                                                    if(tmpdis < tmp_mindis)
                                                    {
														tmp_mindis = tmpdis;
														bestlmap = lmap;
													}
												}
												else{
													if(obstacle_detected){
														break;
													}
												}
											}
										}
                                        // normalize distribution 
										double p1 = exp(-(len-bestlmap)*(len-bestlmap)/(2*measurement_convriance*measurement_convriance)) /
											(2.5*measurement_convriance);//2.5 == sqrt(2*pi)

										if(p1 < EPSILON_ERROR) p1 = EPSILON_ERROR;
										p = p * p1;
									}
								}
							}
						}

						noisepos[j].weight = noisepos[j].weight * p;
					}
				}




				// process after the particle filters 
				double totalweight = 0.0;
				for(int i=0;i<SAMPLENO;i++){
					totalweight = totalweight + noisepos[i].weight;
				}

                // normalize the weight of particles after updating the weight of particles 
				int max_particle_index = 0;
				maxweight = 0.0;
				for(int i=0;i<SAMPLENO;i++)
                {
					noisepos[i].weight = noisepos[i].weight / totalweight;

					if(noisepos[i].weight > maxweight){
						maxweight = noisepos[i].weight;
						max_particle_index = i;
					}
				}

				//resample
				{
					R >> temprand;
					int index = cvRound(temprand * SAMPLENO);
					if(index >= SAMPLENO) index = SAMPLENO;

					double beta = 0.0;
					double max_weight = 0.0;
					int max_weight_index = 0 ;
					//search for the most weight particle 
                    for(int i = 0 ; i< SAMPLENO; i++){
						if(max_weight < noisepos[i].weight){
							max_weight_index = i;
							max_weight = noisepos[i].weight;
						}
					}
					double double_max_weight = 2 * max_weight;

                    //resample operation
					for(int j = 0 ;j< SAMPLENO; j++)
                    {
						R >> temprand ;
						beta = beta + double_max_weight * temprand;

						while(noisepos[index].weight < beta)
                        {
							beta = beta - noisepos[index].weight;
							index = index + 1;
							if(index == SAMPLENO)
								index = 0;
						}

						vpose[j].theta = noisepos[index].theta;
						vpose[j].x = noisepos[index].x;
						vpose[j].y = noisepos[index].y;
						vpose[j].weight = 1.0 / SAMPLENO;
					}
				}

				// calculate the distribution of particles
				double x_e = 0.0, y_e = 0.0;
				for(int i = 0 ; i < SAMPLENO ; i++){
					x_e = x_e + vpose[i].x;
					y_e = y_e + vpose[i].y;
				}
				x_e = x_e / SAMPLENO;
				y_e = y_e / SAMPLENO;

				std_err = 0.0;
				for(int i =0 ; i < SAMPLENO ; i++){
					double dx = vpose[i].x - x_e;
					double dy = vpose[i].y - y_e;
					std_err = std_err + sqrt(dx * dx + dy * dy);
				}
				std_err = std_err / SAMPLENO;

				x_minrange = x_e - std_err;
				x_maxrange = x_e + std_err;
				y_minrange = y_e - std_err;
				y_maxrange = y_e + std_err;

				//
				double deltatheta_e = 0.0;
				for(int i =0 ; i < SAMPLENO ; i++){
					double deltatheta = vpose[i].theta - m_sVehicleStateToNet.f_slamcorrected_theta;

					while(deltatheta > pi)
						deltatheta = deltatheta - pi * 2;
					while(deltatheta <= -pi)
						deltatheta = deltatheta + pi * 2;

					deltatheta_e = deltatheta_e + deltatheta / SAMPLENO;
				}
				double theta_e = m_sVehicleStateToNet.f_slamcorrected_theta + deltatheta_e;

				if(std_err <particle_position_std_threld ){
					//we modified the localized slam result only when results of particle filter is well
					m_sVehicleStateToNet.f_slamcorrected_east = x_e;
					m_sVehicleStateToNet.f_slamcorrected_north = y_e;
					//m_sVehicleStateToNet.f_slamcorrected_theta =  theta_e;
                    m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.fTheta;

				}
                else
                {
                    m_sVehicleStateToNet.f_slamcorrected_east  =m_sVehicleStateToNet.fDRPos_E ;
			        m_sVehicleStateToNet.f_slamcorrected_north  =m_sVehicleStateToNet.fDRPos_N ;
                    m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.fTheta;
                }
            }

		}


}

void pfSlam::Positioning_by_PF_Odometer(points2D* frame_ptr)
{
	float delta_odometer = m_sVehicleStateToNet.fOdometer - m_sVehicleStateToNet.Odometer_pre;
			if(delta_odometer > 100.0f)
				delta_odometer = 0.0f;

			m_sVehicleStateToNet.Odometer_pre = m_sVehicleStateToNet.fOdometer;

			float delta_theta = 0;
			if(fabs(m_sVehicleStateToNet.fRadius) < 1000.0f && fabs(m_sVehicleStateToNet.fRadius) > EPSILON_ERROR)
				delta_theta = delta_odometer / m_sVehicleStateToNet.fRadius;

			m_sVehicleStateToNet.Odometer_theta = m_sVehicleStateToNet.Odometer_theta + delta_theta;
			if(m_sVehicleStateToNet.Odometer_theta > pi *2)
				m_sVehicleStateToNet.Odometer_theta = m_sVehicleStateToNet.Odometer_theta - pi *2;
			else if(m_sVehicleStateToNet.Odometer_theta <= 0)
				m_sVehicleStateToNet.Odometer_theta = m_sVehicleStateToNet.Odometer_theta + pi *2;

			float delta_x = delta_odometer * cos(m_sVehicleStateToNet.Odometer_theta);
			float delta_y = delta_odometer * sin(m_sVehicleStateToNet.Odometer_theta);

			m_sVehicleStateToNet.Odometer_x = m_sVehicleStateToNet.Odometer_x + delta_x;
			m_sVehicleStateToNet.Odometer_y = m_sVehicleStateToNet.Odometer_y + delta_y;

			m_sVehicleStateToNet.f_slamcorrected_east = m_sVehicleStateToNet.f_slamcorrected_east + delta_x;
			m_sVehicleStateToNet.f_slamcorrected_north = m_sVehicleStateToNet.f_slamcorrected_north + delta_y;
			m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.f_slamcorrected_theta + delta_theta;
			if(m_sVehicleStateToNet.f_slamcorrected_theta > pi *2)
				m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.f_slamcorrected_theta - pi *2;
			else if(m_sVehicleStateToNet.Odometer_theta <= 0)
				m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.f_slamcorrected_theta + pi *2;


			//
			double maxweight = 0.0;
			x_minrange = INVALIDVALUE;
			x_maxrange = -INVALIDVALUE;
			y_minrange = INVALIDVALUE;
			y_maxrange = - INVALIDVALUE;

			if(1){
				MSLRandomSource R;
				double temprand;

				///localization
				if(!filter_initialized){

					//initial the particles
					for(int i=0;i<SAMPLENO;i++){
						noisepos[i] = ResetPartical(m_sVehicleStateToNet.f_slamcorrected_east,
							m_sVehicleStateToNet.f_slamcorrected_north, m_sVehicleStateToNet.f_slamcorrected_theta);
					}


					filter_initialized = true;
				}
				else{
					// pass the result of resamppling to noisepos , the bring in 10% new particle
					for(int j = 0 ;j< cvRound(SAMPLENO * 0.9); j++){
						noisepos[j].theta = vpose[j].theta;
						noisepos[j].x = vpose[j].x;
						noisepos[j].y = vpose[j].y;
						noisepos[j].weight = vpose[j].weight;
					}
					for(int j =cvRound(SAMPLENO * 0.9); j < SAMPLENO ; j++){
						noisepos[j] = ResetPartical(m_sVehicleStateToNet.f_slamcorrected_east,
							m_sVehicleStateToNet.f_slamcorrected_north, m_sVehicleStateToNet.f_slamcorrected_theta);
					}

					// vehicle motion updating
					for(int j = 0 ; j< SAMPLENO; j++){
						//R >> temprand;
						//noisepos[j].theta = noisepos[j].theta + delta_theta + move_theta_std * NormalDistribute();
						noisepos[j].theta= m_sVehicleStateToNet.f_slamcorrected_theta;
                        while(noisepos[j].theta >= 2 * pi)
							noisepos[j].theta = noisepos[j].theta - 2 * pi;
						while(noisepos[j].theta < 0)
							noisepos[j].theta = noisepos[j].theta + 2 * pi;

						double costheta = cos(noisepos[j].theta);
						double sintheta = sin(noisepos[j].theta);
						double delta_s_sample = move_std * NormalDistribute();
						//R >> temprand;
						//float delta_s_sample = move_std * 2 * (temprand - 0.5);
						noisepos[j].x = noisepos[j].x + (delta_odometer + delta_s_sample) * costheta;
						noisepos[j].y = noisepos[j].y + (delta_odometer + delta_s_sample) * sintheta;
					}

					{
						//write noisepose to share memory
	/*#ifdef USESHAREMEMORYDISPLAYSLAM
						int initpos = 0;
						LARGE_INTEGER lTime1 = GetHighAccurateTimeTick();
						memcpy(m_sharedmemorymanagement_DisplaySlam.Buf1 + initpos, &lTime1.QuadPart, sizeof(LARGE_INTEGER));
						initpos += sizeof(LARGE_INTEGER);
						memcpy(m_sharedmemorymanagement_DisplaySlam.Buf1 + initpos, (char*)noisepos, sizeof(pose) * SAMPLENO);
			pfslam.cpp			m_sharedmemorymanagement_DisplaySlam.Copy2ShareMemoryDisplaySlam1();
	#endif*/
					}
				}

				// update the weight of particles according to the result of mapping
				{
					for(int j = 0 ; j< SAMPLENO; j++){
						//
						double p = 1.0;

						pos sensorpos_global ;
						sensorpos_global.x = noisepos[j].x + cos(noisepos[j].theta) * VEHICLELEN_M;
						sensorpos_global.y = noisepos[j].y + sin(noisepos[j].theta) * VEHICLELEN_M;

						{
							for(int n=0; n < raytracing_angle_count; n++){
								double serangle = n * raytracing_angleresolution;

								double err = serangle - (noisepos[j].theta) * 180 / pi;
								while(err >= 360)
									err = err-360;

								while(err < 0)
									err = err+360;

								if(err <= 95 || err >= 265){

									unsigned int index=0;
									if(err <= 95){
										index = cvRound(err / SICKANGLERESOLUTION) + 381;
									}
									else{
										index=cvRound((err - 360.0f) / SICKANGLERESOLUTION) + 381;
									}

									double len=frame_ptr->distance[index];

									{
										int x,y;
										Convert_World_to_OGM(sensorpos_global.x - slammap_center_pos.x, sensorpos_global.y - slammap_center_pos.y, &x, &y,
											(slamogmsize - 1) / 2 * globalmapresolution, (slamogmsize - 1) / 2 * globalmapresolution, globalmapresolution);

										double bestlmap = 1000.0f, tmp_mindis =1000.0f;

										bool obstacle_detected = false;

										int* raytracing_x_array_startptr = &raytracing_x_array[n * raytracing_lmap_count];
										int* raytracing_y_array_startptr = &raytracing_y_array[n * raytracing_lmap_count];
										for(int i = 0 ; i < raytracing_lmap_count; i++){
											int deltax = *(raytracing_x_array_startptr + i);
											int deltay = *(raytracing_y_array_startptr + i);
											int x1 = x + deltax;
											int y1 = y + deltay;

											float lmap = i * globalmapresolution;

											if(x1>=0 && x1<slamogmsize && y1>=0 && y1<slamogmsize){

												if(prob_global[x1 + y1*slamogmsize]  > MAXPROB - EPSILON_ERROR){
													obstacle_detected = true;
													double tmpdis = fabs(lmap - len);
													if(tmpdis < tmp_mindis){
														tmp_mindis = tmpdis;
														bestlmap = lmap;
													}
												}
												else{
													if(obstacle_detected){
														break;
													}
												}
											}
										}

										double p1 = exp(-(len-bestlmap)*(len-bestlmap)/(2*measurement_convriance*measurement_convriance)) /
											(2.5*measurement_convriance);//2.5 == sqrt(2*pi)

										if(p1 < EPSILON_ERROR) p1 = EPSILON_ERROR;
										p = p * p1;
									}
								}
							}
						}

						noisepos[j].weight = noisepos[j].weight * p;
					}
				}

				//  post process of particle filter
				double totalweight = 0.0;
				for(int i=0;i<SAMPLENO;i++){
					totalweight = totalweight + noisepos[i].weight;
				}

				int max_particle_index = 0;
				maxweight = 0.0;
				for(int i=0;i<SAMPLENO;i++){
					noisepos[i].weight = noisepos[i].weight / totalweight;

					if(noisepos[i].weight > maxweight){
						maxweight = noisepos[i].weight;
						max_particle_index = i;
					}
				}

				// resampple
				{
					R >> temprand;
					int index = cvRound(temprand * SAMPLENO);
					if(index >= SAMPLENO) index = SAMPLENO;

					double beta = 0.0;
					double max_weight = 0.0;
					int max_weight_index = 0;
					for(int i = 0 ; i< SAMPLENO; i++){
						if(max_weight < noisepos[i].weight){
							max_weight_index = i;
							max_weight = noisepos[i].weight;
						}
					}
					double double_max_weight = 2 * max_weight;

					for(int j = 0 ;j< SAMPLENO; j++){
						R >> temprand ;
						beta = beta + double_max_weight * temprand;

						while(noisepos[index].weight < beta){
							beta = beta - noisepos[index].weight;
							index = index + 1;
							if(index == SAMPLENO)
								index = 0;
						}

						vpose[j].theta = noisepos[index].theta;
						vpose[j].x = noisepos[index].x;
						vpose[j].y = noisepos[index].y;
						vpose[j].weight = 1.0 / SAMPLENO;
					}
				}

				//calculate the distribution of particles
				float x_e = 0.0f, y_e = 0.0f;
				for(int i =0 ; i < SAMPLENO ; i++){
					x_e = x_e + vpose[i].x ;
					y_e = y_e + vpose[i].y ;
				}
				x_e = x_e / SAMPLENO;
				y_e = y_e / SAMPLENO;

				std_err = 0.0f;
				for(int i =0 ; i < SAMPLENO ; i++){
					float dx = vpose[i].x - x_e;
					float dy = vpose[i].y - y_e;

					std_err = std_err + sqrt(dx * dx + dy * dy);
				}
				std_err = std_err / SAMPLENO;

				x_minrange = x_e - std_err;
				x_maxrange = x_e + std_err;
				y_minrange = y_e - std_err;
				y_maxrange = y_e + std_err;

				//calculate of the average orientation
				float deltatheta_e = 0.0f;
				for(int i =0 ; i < SAMPLENO ; i++){
					float deltatheta = vpose[i].theta - m_sVehicleStateToNet.f_slamcorrected_theta;

					while(deltatheta > pi)
						deltatheta = deltatheta - pi * 2;
					while(deltatheta <= -pi)
						deltatheta = deltatheta + pi * 2;

					deltatheta_e = deltatheta_e + deltatheta / SAMPLENO;
				}
				float theta_e = m_sVehicleStateToNet.f_slamcorrected_theta + deltatheta_e;

				if(std_err <particle_position_std_threld ){
					//only use the result of slam when the result is good
					m_sVehicleStateToNet.f_slamcorrected_east = x_e;
					m_sVehicleStateToNet.f_slamcorrected_north = y_e;
					m_sVehicleStateToNet.f_slamcorrected_theta =  theta_e;
				}
			}

			{
				//vpose + others    write the vpose and others to share memory
	#ifdef USESHAREMEMORYDISPLAYSLAM

	/*			int initpos = 0;
				LARGE_INTEGER lTime2 = GetHighAccurateTimeTick();
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &lTime2.QuadPart, sizeof(LARGE_INTEGER));
				initpos += sizeof(LARGE_INTEGER);

				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, vpose, sizeof(pose)*SAMPLENO);
				initpos+=sizeof(pose)*SAMPLENO;
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &m_sVehicleStateToNet, sizeof(SVehicleStateToNet));
				initpos+=sizeof(SVehicleStateToNet);
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &slammap_center_pos, sizeof(pos));
				initpos+=sizeof(pos);
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &x_minrange, sizeof(float));
				initpos+=sizeof(float);
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &x_maxrange, sizeof(float));
				initpos+=sizeof(float);
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &y_minrange, sizeof(float));
				initpos+=sizeof(float);
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &y_maxrange, sizeof(float));
				initpos+=sizeof(float);
				memcpy(m_sharedmemorymanagement_DisplaySlam.Buf2 + initpos, &std_err, sizeof(float));

				m_sharedmemorymanagement_DisplaySlam.Copy2ShareMemoryDisplaySlam2();
*/
	#endif
			}
}

void pfSlam::MappingPreProc(void)
{

	    state_struct sensor_pose;
		sensor_pose.position.x=m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M*cos(m_sVehicleStateToNet.f_slamcorrected_theta);
		sensor_pose.position.y=m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M*sin(m_sVehicleStateToNet.f_slamcorrected_theta);
		sensor_pose.theta=m_sVehicleStateToNet.f_slamcorrected_theta;

		{
			//tranlate the global map accord to needs.
			if(fabs(m_sVehicleStateToNet.f_slamcorrected_east-slammap_center_pos.x)>20 || fabs(m_sVehicleStateToNet.f_slamcorrected_north-slammap_center_pos.y)>20){
				int deltax=cvRound((m_sVehicleStateToNet.f_slamcorrected_east-slammap_center_pos.x) / globalmapresolution);
				int deltay=cvRound((m_sVehicleStateToNet.f_slamcorrected_north-slammap_center_pos.y) / globalmapresolution);

				slammap_center_pos.x=m_sVehicleStateToNet.f_slamcorrected_east;
				slammap_center_pos.y=m_sVehicleStateToNet.f_slamcorrected_north;


				memcpy(tmp_prob_global,prob_global,sizeof(double)*slamogmsize*slamogmsize);
				memcpy(tmp_prob_global_static,prob_global_static,sizeof(double)*slamogmsize*slamogmsize);
				//initialize global prob map
				for(int i=0;i<slamogmsize;i++)
					for(int j=0;j<slamogmsize;j++){
						prob_global[i*slamogmsize+j] = UNKNOWNPROB;// unknown

						prob_global_static[i*slamogmsize+j] = UNKNOWNPROB;//Î´Öª  unknown
					}

					if(deltax>=0){
						if(deltay>=0){
							for(int i=0;i<slamogmsize-deltay;i++)
								for(int j=0;j<slamogmsize-deltax;j++){
									prob_global[i*slamogmsize+j]=tmp_prob_global[(i+deltay)*slamogmsize+(j+deltax)];

									prob_global_static[i*slamogmsize+j]=tmp_prob_global_static[(i+deltay)*slamogmsize+(j+deltax)];
								}
						}
						else{
							for(int i=0;i<slamogmsize-(-deltay);i++)
								for(int j=0;j<slamogmsize-deltax;j++){
									prob_global[(i-deltay)*slamogmsize+j]=tmp_prob_global[i*slamogmsize+(j+deltax)];

									prob_global_static[(i-deltay)*slamogmsize+j]=tmp_prob_global_static[i*slamogmsize+(j+deltax)];
								}
						}
					}
					else{
						if(deltay>=0){
							for(int i=0;i<slamogmsize-deltay;i++)
								for(int j=0;j<slamogmsize-(-deltax);j++){
									prob_global[i*slamogmsize+(j-deltax)]=tmp_prob_global[(i+deltay)*slamogmsize+j];

									prob_global_static[i*slamogmsize+(j-deltax)]=tmp_prob_global_static[(i+deltay)*slamogmsize+j];
								}
						}
						else{
							for(int i=0;i<slamogmsize-(-deltay);i++)
								for(int j=0;j<slamogmsize-(-deltax);j++){
									prob_global[(i-deltay)*slamogmsize+(j-deltax)]=tmp_prob_global[i*slamogmsize+j];

									prob_global_static[(i-deltay)*slamogmsize+(j-deltax)]=tmp_prob_global_static[i*slamogmsize+j];
								}
						}
					}

					//pass the single frame map 
					for(int i = 0 ; i < slamogmsize*slamogmsize ; i++)
					{
						if(prob_global_static[i] > MAXPROB - EPSILON_ERROR ||
							prob_global_static[i] < MINPROB + EPSILON_ERROR)
							prob_global[i] = prob_global_static[i];
					}
			}
			else
			{
				// pass the singe frame map 
				for(int i = 0 ; i < slamogmsize*slamogmsize ; i++)
				{
					if(prob_global_static[i] > MAXPROB - EPSILON_ERROR ||
						prob_global_static[i] < MINPROB + EPSILON_ERROR)
						prob_global[i] = prob_global_static[i];
				}
			}
		}

}


void pfSlam::MappingUpdate(points2D* frame_ptr)
{

	state_struct sensor_pose;
	sensor_pose.position.x=m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M*cos(m_sVehicleStateToNet.f_slamcorrected_theta);
	sensor_pose.position.y=m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M*sin(m_sVehicleStateToNet.f_slamcorrected_theta);
	sensor_pose.theta=m_sVehicleStateToNet.f_slamcorrected_theta;


		//update the global map with measure data, consider the error
	if(1){
		pos_int sensor_pose_incell;
        //cout<<"mapping Update sensor_pose:"<<sensor_pose.position.x<<" " <<sensor_pose.position.y<<endl;

		Convert_World_to_OGM(sensor_pose.position.x - slammap_center_pos.x, sensor_pose.position.y - slammap_center_pos.y, &sensor_pose_incell.x, &sensor_pose_incell.y,
			(slamogmsize-1) / 2 * globalmapresolution, (slamogmsize-1) / 2 * globalmapresolution, globalmapresolution);

		for(int i=0;i<slamogmsize;i++){
			for(int j=0;j<slamogmsize;j++){

				double tmpdis = sqrt( pow((double)(sensor_pose_incell.x - j) , 2) + pow((double)(sensor_pose_incell.y - i) , 2)) * globalmapresolution;
				if(tmpdis > 50.0)
					continue;

				//absolute global position of cell
				pos cell_pos_m;
				Convert_OGM_to_World(j, i, &cell_pos_m.x, &cell_pos_m.y,
					(slamogmsize-1) / 2 * globalmapresolution, (slamogmsize-1) / 2 * globalmapresolution, globalmapresolution);
				cell_pos_m.x = cell_pos_m.x + slammap_center_pos.x;
				cell_pos_m.y = cell_pos_m.y + slammap_center_pos.y;

        

				
                int laserindex = -1;
				double dis = 100;
				double deltaphi = 0;
				bool res = IsCellInCone(sensor_pose.position.x,sensor_pose.position.y,sensor_pose.theta,
					cell_pos_m.x,cell_pos_m.y, forward_laser_index,
					&laserindex, &dis, &deltaphi);

				if(res){
					//add some noise to the angles
					double measurement = frame_ptr->distance[laserindex];
					int sickanglerange = cvRound(sickangle_err_std / SICKANGLERESOLUTION);
					for(int t = MAX(0, laserindex - sickanglerange); t < MIN(laserindex + sickanglerange, SICKPTNUM); t++)
						measurement = MIN(measurement, frame_ptr->distance[t]);

					double prob_m_z = PROB_EMP;//default

					//
					double epsilon=sickrange_err_std;
					if(measurement > 0.5){
						if(measurement - dis > epsilon)//cell closer
							prob_m_z = PROB_EMP;
						else if(dis - measurement > epsilon)//cell further
							prob_m_z = PROB_UNKNOWN;
						else//cell hit
							prob_m_z = PROB_OCC;
					}
					else
						prob_m_z = PROB_UNKNOWN;

					double s = prob_m_z / (1 - prob_m_z) * prob_global[i*slamogmsize+j] / (1 - prob_global[i*slamogmsize+j]);
					double prob = s / ( 1 + s);

					if(prob > 1.0 - EPSILON_ERROR)
						prob = 1.0 - EPSILON_ERROR;
					if(prob < EPSILON_ERROR)
						prob = EPSILON_ERROR;

                    //for test 
					prob_global[i*slamogmsize+j]=prob;
					//prob_global[i*slamogmsize+j]=PROB_EMP;
				}
			}
		}
	}
}

void pfSlam::MappingPostProc(points2D* frame_ptr)
{

	state_struct sensor_pose;
		sensor_pose.position.x=m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M*cos(m_sVehicleStateToNet.f_slamcorrected_theta);
		sensor_pose.position.y=m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M*sin(m_sVehicleStateToNet.f_slamcorrected_theta);
		sensor_pose.theta=m_sVehicleStateToNet.f_slamcorrected_theta;

		//
		if(1){
			int sensorpos_local=cvRound((VEHICLEPOSINOGM_Y_M + VEHICLELEN_M) / OGMRESOLUTION);
			memset(prob_local, 0, sizeof(double) * OGMHEIGHT_CELL * OGMWIDTH_CELL);
			int top_row=MIN(OGMHEIGHT_CELL, cvRound((VEHICLEPOSINOGM_Y_M + VEHICLELEN_M) / OGMRESOLUTION));
			//for(int i=0; i<top_row; i++){
			/*for(int i=0; i<OGMHEIGHT_CELL; i++){
				for(int j=0; j<OGMWIDTH_CELL; j++){
					pos cell_pos_local;
					Convert_OGM_to_World(j, i, &cell_pos_local.x, &cell_pos_local.y,
						(OGMWIDTH_CELL-1) / 2 * OGMRESOLUTION, sensorpos_local * OGMRESOLUTION, OGMRESOLUTION);

					pos cell_pos_global ;
					cell_pos_global.x=sensor_pose.position.x+sin(sensor_pose.theta)*cell_pos_local.x+cos(sensor_pose.theta)*cell_pos_local.y;

					//cell_pos_global.y=sensor_pose.position.y-cos(sensor_pose.theta)*cell_pos_local.x+sin(sensor_pose.theta)*cell_pos_local.y;
					//for test
                    cell_pos_global.y=sensor_pose.position.y+cos(sensor_pose.theta)*cell_pos_local.x+sin(sensor_pose.theta)*cell_pos_local.y;
					int i1, j1;
					Convert_World_to_OGM(cell_pos_global.x - slammap_center_pos.x, cell_pos_global.y - slammap_center_pos.y, &j1, &i1,
						(slamogmsize-1) / 2 * globalmapresolution, (slamogmsize-1) / 2 * globalmapresolution, globalmapresolution);

					if(i1 >= 0 && i1 < slamogmsize && j1 >= 0 && j1< slamogmsize)
						prob_local[i * OGMWIDTH_CELL + j] = prob_global[i1 * slamogmsize + j1];
				}
			}*/
		}

		//prob_global_frame  for display
		{
			for(int i=0;i<slamogmsize;i++)
				for(int j=0;j<slamogmsize;j++)
					prob_global_frame[i*slamogmsize+j] = MINPROB;

			state_struct sensor_pose;
			sensor_pose.position.x=m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M * cos(m_sVehicleStateToNet.fTheta);
			sensor_pose.position.y=m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M * sin(m_sVehicleStateToNet.fTheta);
			sensor_pose.theta=m_sVehicleStateToNet.f_slamcorrected_theta;
			for(int i=0;i<SICKPTNUM;i++){
				if(frame_ptr->distance[i]>0.3 && frame_ptr->distance[i]<50.0){
					//record the pose of measure point
					double laser_angle = (m_sVehicleStateToNet.f_slamcorrected_theta + (i * SICKANGLERESOLUTION + STARTANGLE) * pi / 180) - pi / 2 ;
					int x = cvRound((sensor_pose.position.x + frame_ptr->distance[i] * cos(laser_angle) - slammap_center_pos.x) / globalmapresolution + (slamogmsize - 1)/2);
					int y = cvRound((sensor_pose.position.y + frame_ptr->distance[i] * sin(laser_angle) - slammap_center_pos.y) / globalmapresolution + (slamogmsize - 1)/2);
					if(x >= 0 && x < slamogmsize && y >= 0 && y < slamogmsize)
						prob_global_frame[y*slamogmsize+x]=MAXPROB;
				}
			}
            //double test_angle = (m_sVehicleStateToNet.f_slamcorrected_theta+(380*SICKANGLERESOLUTION+STARTANGLE)*pi/180)-pi/2;
            //cout<<"laser angle:"<<test_angle<<endl;
            //cout<<"compare vehicles heading:"<<m_sVehicleStateToNet.f_slamcorrected_theta<<endl;
		}


}


void pfSlam::Mapping_StaticMapBuilding(points2D* frame_ptr)
{
    cout<<" static map building is running"<<endl; 
    //initialize the global static gridmap
	for(int i=0;i<slamogmsize;i++)
			for(int j=0;j<slamogmsize;j++)
				prob_global_static[i*slamogmsize+j] = UNKNOWNPROB;//all empty

		state_struct sensor_pose;
		sensor_pose.position.x=m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M*cos(m_sVehicleStateToNet.f_slamcorrected_theta);
		sensor_pose.position.y=m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M*sin(m_sVehicleStateToNet.f_slamcorrected_theta);
		sensor_pose.theta=m_sVehicleStateToNet.f_slamcorrected_theta;

		//
		if(1){
			pos_int sensor_pose_incell;
            //slammap_center_pos.x = sensor_pose.position.x;
            //slammap_center_pos.y = sensor_pose.position.y;

			Convert_World_to_OGM((sensor_pose.position.x - slammap_center_pos.x), (sensor_pose.position.y - slammap_center_pos.y), &sensor_pose_incell.x, &sensor_pose_incell.y,
				(slamogmsize-1) / 2 * globalmapresolution, (slamogmsize-1) / 2 * globalmapresolution, globalmapresolution);

            //cout<<"sensor_pose_incell:"<<sensor_pose_incell.x<<"  "<<sensor_pose_incell.y<<endl;
            //cout<<"sensor_pose:"<<sensor_pose.position.x<<" "<<sensor_pose.position.y<<endl; 
            cout<<"slammap_center:"<<slammap_center_pos.x<<" "<<slammap_center_pos.y<<endl; 

			for(int i=0;i<slamogmsize;i++){
				for(int j=0;j<slamogmsize;j++){

					double tmpdis = sqrt( pow((double)(sensor_pose_incell.x - j) , 2) + pow((double)(sensor_pose_incell.y - i) , 2)) * globalmapresolution;
					if(tmpdis > 50.0) //check if the point is lacated outside the slamogmmap
						continue;
                    
					//absolute global position of cell
					pos cell_pos_m;
					Convert_OGM_to_World(j, i, &cell_pos_m.x, &cell_pos_m.y,
						(slamogmsize-1) / 2 * globalmapresolution, (slamogmsize-1) / 2 * globalmapresolution, globalmapresolution);
					cell_pos_m.x = cell_pos_m.x + slammap_center_pos.x;
					cell_pos_m.y = cell_pos_m.y + slammap_center_pos.y;

					int laserindex = -1;
				    double dis = 100;
					double deltaphi = 0;
					bool res = IsCellInCone(sensor_pose.position.x,sensor_pose.position.y,sensor_pose.theta,
						cell_pos_m.x,cell_pos_m.y, forward_laser_index,
						&laserindex, &dis, &deltaphi);
                    //cout<<"forward_laser_index:"<<forward_laser_index<<endl;
                    

                    // calculate the distance of cell to sensor pose , get the laserindex 

					if(res){
						//add heading noise to the localization
						double measurement = frame_ptr->distance[laserindex];
						double sickanglerange = sickangle_err_std2 / SICKANGLERESOLUTION;// one degree error 
                        // find the minimum distance between -1 to 1 degree around cerntain index  as the measurement  
						for(int t = MAX(0, laserindex - sickanglerange); t < MIN(laserindex + sickanglerange, SICKPTNUM); t++)
							measurement = MIN(measurement, frame_ptr->distance[t]);


						// this is a distance error  , 2 meters for distance
						double epsilon=sickrange_err_std2;
						if(measurement > 0.5){
							if(measurement - dis > epsilon)//cell closer
								prob_global_static[i*slamogmsize+j] = MINPROB;
							else if(dis - measurement > epsilon)//cell further
								prob_global_static[i*slamogmsize+j] = UNKNOWNPROB;
							else//cell hit
								prob_global_static[i*slamogmsize+j] = MAXPROB;
						}
						else
							prob_global_static[i*slamogmsize+j]=UNKNOWNPROB;

					}
				}
			}
		}

		// initialize global map with static map 
		memcpy(prob_global, prob_global_static, sizeof(double) * slamogmsize*slamogmsize);

		//center of map should be the pose of car 
		slammap_center_pos.x = m_sVehicleStateToNet.f_slamcorrected_east;
		slammap_center_pos.y = m_sVehicleStateToNet.f_slamcorrected_north;

        cout<<"static map slam map center:"<<slammap_center_pos.x<<" "<<slammap_center_pos.y<<endl;
		
        //laser data of single frame, prob_global_frame for display
		{
			for(int i=0;i<slamogmsize;i++)
				for(int j=0;j<slamogmsize;j++)
					prob_global_frame[i*slamogmsize+j] = MINPROB;

			state_struct sensor_pose;
			sensor_pose.position.x=m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M * cos(m_sVehicleStateToNet.fTheta);
			sensor_pose.position.y=m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M * sin(m_sVehicleStateToNet.fTheta);
			sensor_pose.theta=m_sVehicleStateToNet.f_slamcorrected_theta;
			for(int i=0;i<SICKPTNUM;i++){
				if(frame_ptr->distance[i]>0.3 && frame_ptr->distance[i]<50.0){
					//record the pose of measurement point after tranforming
					double laser_angle = (m_sVehicleStateToNet.f_slamcorrected_theta + (i * SICKANGLERESOLUTION + STARTANGLE) * pi / 180) - pi / 2 ;
				    //double laser_angle = (m_sVehicleStateToNet.f_slamcorrected_theta + (i * SICKANGLERESOLUTION + STARTANGLE) *pi /180);
                    int x = cvRound((sensor_pose.position.x + frame_ptr->distance[i] * cos(laser_angle) - slammap_center_pos.x) / globalmapresolution + (slamogmsize - 1)/2);
					int y = cvRound((sensor_pose.position.y + frame_ptr->distance[i] * sin(laser_angle) - slammap_center_pos.y) / globalmapresolution + (slamogmsize - 1)/2);
					if(x >= 0 && x < slamogmsize && y >= 0 && y < slamogmsize)
						prob_global_frame[y*slamogmsize+x]=MAXPROB;
				}
			}
        }


        //cout<<"m_sVehicleStateToNet.f_slamcorrected_east:"<<m_sVehicleStateToNet.f_slamcorrected_east<<endl;
        //cout<<"m_sVehicleStateToNet.f_slamcorrected_north:"<<m_sVehicleStateToNet.f_slamcorrected_north<<endl;
        //cout<<" static map building is finished "<<endl;
		localization_initialized = true;
}






void pfSlam::publishTransform()
{
    map_to_odom_mutex_.lock();
    ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_delay_);
    tfB_->sendTransform(tf::StampedTransform (map_to_odom_, tf_expiration, map_frame_, odom_frame_));
    map_to_odom_mutex_.unlock();

}

bool pfSlam::mapCallback(nav_msgs::GetMap::Request &req,
        nav_msgs::GetMap::Response &res)
{
    boost::mutex::scoped_lock map_lock (map_mutex_);
    if(got_map_ && map_.map.info.width && map_.map.info.height)
    {
        res = map_ ; 
        return true;
    }
    else
        return false; 
}

void pfSlam::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static ros::Time last_map_update(0,0);
    //how to use tf listenner or filter to get the transforms 
    ros::Time scan_time = scan->header.stamp;
    //get the pose of hokuyo in odometry frame  
    //tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)), scan_time, laser_frame_);
    //tf::Stamped<tf::Transform> odom_pose;
    tf::Stamped<tf::Transform> vehicle_pose;

    //get the car's pose relative to odom frame
    tf::Stamped<tf::Pose> ident_base(tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)), scan_time, base_frame_);
    try
    {
        tf_.transformPose(odom_frame_,ident_base,vehicle_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)",e.what());
    }
    // have gotten the pose of the car in odometry frame
    //debug 
    double x,y, theta; 
    x = vehicle_pose.getOrigin().x();
    y = vehicle_pose.getOrigin().y();
    theta = tf::getYaw(vehicle_pose.getRotation());
    while( theta < 0)
        theta = theta+pi*2;
    while (theta >= pi*2)
        theta = theta -pi*2;


    // update integrated pose information
    m_sVehicleStateToNet.f_integrated_east = x; 
    m_sVehicleStateToNet.f_integrated_north = y; 
    m_sVehicleStateToNet.fTheta = theta; 

    // update dr pose information 
    m_sVehicleStateToNet.fDRPos_E = x; 
    m_sVehicleStateToNet.fDRPos_N = y;

    //think poses of dr and gps are accurate for now  


    //m_sVehicleStateToNet.f_slamcorrected_east = m_sVehicleStateToNet.f_integrated_east;
    //m_sVehicleStateToNet.f_slamcorrected_north = m_sVehicleStateToNet.f_integrated_north;
    //m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.fTheta; 

    
    bool useINSheading = true;
    bool useSLAM = true; 
    //cout<<" useSLAM and useINSheading."<<endl;
    

    // get laser scan data and fill the data in the frame
    int num_ranges = scan->ranges.size();
    
    //double* ranges_double = new double[num_ranges];
    for(int i=0; i < num_ranges; i ++)
    {
        //Must filter out short readings, because the mapper won't 
        if(scan->ranges[i]< scan->range_min)
            frame.distance[i] = (double)scan->range_max;
        else
            frame.distance[i] = (double)scan->ranges[i];
       // cout<<"num:"<<i<<" "<<frame.distance[i]<<endl;
    }

    //cout << "lacalization:" << localization_initialized <<endl; 
    if(!localization_initialized)
    {

        m_sVehicleStateToNet.f_slamcorrected_east = m_sVehicleStateToNet.f_integrated_east;
        m_sVehicleStateToNet.f_slamcorrected_north = m_sVehicleStateToNet.f_integrated_north;  
        m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.fTheta;

        INSpose_cur.position.x = x;
        INSpose_cur.position.y = y;
        INSpose_cur.theta = theta; 
        
        INSpose_pre.position.x = x; 
        INSpose_pre.position.y = y; 
        INSpose_pre.theta = theta; 
        // if useINSheading is true, dont use odometer below 
       // m_sVehicleStateToNet.Odometer_x = x;
       // m_sVehicleStateToNet.Odometer_y = y; 
       // m_sVehicleStateToNet.Odometer_theta = theta; 

       // cout<<"first heading:"<< m_sVehicleStateToNet.f_slamcorrected_theta<<endl;
       // cout<<"slamcorrected:"<<m_sVehicleStateToNet.f_slamcorrected_east<<" "<<m_sVehicleStateToNet.f_slamcorrected_north<<endl;
        Mapping_StaticMapBuilding(&frame);

        cout<<"fisrt time slammap center:"<<slammap_center_pos.x<<" "<<slammap_center_pos.y<<endl;
        
        //cout<<" just finish the static map building "<<endl;
        //cout<<" INSpose_cur:"<<INSpose_cur.position.x <<" "<<INSpose_cur.position.y<<endl;
        //cout<<" INSpose_pre:"<<INSpose_pre.position.y <<" "<<INSpose_cur.position.y<<endl; 
       
        //cout<<" slam correct pose:"<<m_sVehicleStateToNet.f_slamcorrected_east<<" "<<m_sVehicleStateToNet.f_slamcorrected_north<<" "<<m_sVehicleStateToNet.f_slamcorrected_theta<<endl;

    }
    else
    {

    
        //Positioning_by_PF(&frame);
        //build the static map 
       // Positioning_by_PF_Odometer(&frame);
        //cout<< "East:"<<m_sVehicleStateToNet.fDRPos_E<<endl;
        //cout<< "North:"<<m_sVehicleStateToNet.fDRPos_N<<endl;
        //cout<<"heading:"<<m_sVehicleStateToNet.fTheta<<endl;
        //cout<<"after pf slammap center:"<<slammap_center_pos.x<<" "<<slammap_center_pos.y<<endl;

        Positioning_by_PF(&frame);
        if(0)
        {
            m_sVehicleStateToNet.f_slamcorrected_east = m_sVehicleStateToNet.f_integrated_east;
            m_sVehicleStateToNet.f_slamcorrected_north = m_sVehicleStateToNet.f_integrated_north;
            m_sVehicleStateToNet.f_slamcorrected_theta = m_sVehicleStateToNet.fTheta;

        
        }
        MappingPreProc();
        MappingUpdate(&frame);
        MappingPostProc(&frame);

        //cout<<"center:"<<slammap_center_pos.x<<"  "<<slammap_center_pos.y<<endl;


        //unsigned char* ptr;
        for(int i = 0 ; i <slamogmsize; i ++)
        {
            //ptr = display_src.ptr<unsigned char>(i);
            unsigned char* ptr =display_src.ptr<unsigned char>(i);
            for(int j = 0; j <slamogmsize; j++)
            {
            
                double val = prob_global[i*slamogmsize+j];
                unsigned char t = (unsigned char)((1- val)*255);
                if(val > MAXPROB - EPSILON_ERROR)
                    t= 0;
                else if (val < MINPROB + EPSILON_ERROR)
                    t = 255;
                else
                    t=120;
            //    unsigned char* ptr = display_src.ptr<unsigned char>((slamogmsize-i-1)*display_src.step);
                ptr[3*j] = t;
                ptr[3*j+1] = t;
                ptr[3*j+2] = t;

            }
        }
        //draw all the particles 
        {
            int i,j, i1, j1;
            for (int t = 0; t < SAMPLENO; t++)
            {
                pose particle = noisepos[t];
                int radius=cvRound(particle.weight/(1.0/SAMPLENO));
                radius = MAX(radius,1);
                i = cvRound((particle.y - slammap_center_pos.y)/globalmapresolution)+(slamogmsize-1)/2;
                j = cvRound((particle.x - slammap_center_pos.x)/globalmapresolution)+(slamogmsize-1)/2;
                //circle(display_src,Point(j,slamogmsize-1-i),radius,Scalar(255,0,0),1,8,0);
                circle(display_src,Point(j,i),radius,Scalar(255,0,0),1,8,0);
           }
        }
        //draw car pose ?
        {
            int i, j, i1, j1;
            i = cvRound((m_sVehicleStateToNet.fDRPos_N - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2;
            j = cvRound((m_sVehicleStateToNet.fDRPos_E - slammap_center_pos.x)/globalmapresolution) + (slamogmsize-1)/2;
            state_struct tmpsensor_pos;
            tmpsensor_pos.position.x = m_sVehicleStateToNet.fDRPos_E + VEHICLELEN_M * cos(m_sVehicleStateToNet.fTheta);
            tmpsensor_pos.position.y = m_sVehicleStateToNet.fDRPos_N + VEHICLELEN_M * sin(m_sVehicleStateToNet.fTheta);
            i1 = cvRound((tmpsensor_pos.position.y - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2;
            j1 = cvRound((tmpsensor_pos.position.x - slammap_center_pos.x)/globalmapresolution) + (slamogmsize-1)/2;
            line(display_src,Point(j, i),Point(j1, i1), Scalar(0,255,0),1);
            
            i = cvRound((m_sVehicleStateToNet.f_integrated_north - slammap_center_pos.y)/globalmapresolution)+(slamogmsize-1)/2;
            j = cvRound((m_sVehicleStateToNet.f_integrated_east - slammap_center_pos.x)/globalmapresolution)+(slamogmsize-1)/2;
            tmpsensor_pos;
			tmpsensor_pos.position.x = m_sVehicleStateToNet.f_integrated_east + VEHICLELEN_M * cos(m_sVehicleStateToNet.fTheta);
			tmpsensor_pos.position.y = m_sVehicleStateToNet.f_integrated_north + VEHICLELEN_M * sin(m_sVehicleStateToNet.fTheta);
			i1 = cvRound((tmpsensor_pos.position.y-slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j1 = cvRound((tmpsensor_pos.position.x-slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			line(display_src,cvPoint(j ,i),cvPoint(j1 ,i1),Scalar(0,0,0),2);

			i = cvRound((m_sVehicleStateToNet.Odometer_y - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j = cvRound((m_sVehicleStateToNet.Odometer_x - slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			state_struct sensor_pose1;
			sensor_pose1.position.x = m_sVehicleStateToNet.Odometer_x + VEHICLELEN_M * cos(m_sVehicleStateToNet.Odometer_theta);
			sensor_pose1.position.y = m_sVehicleStateToNet.Odometer_y + VEHICLELEN_M * sin(m_sVehicleStateToNet.Odometer_theta);
			i1 = cvRound((sensor_pose1.position.y-slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j1 = cvRound((sensor_pose1.position.x-slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			line(display_src,cvPoint(j ,i),cvPoint(j1 ,i1),Scalar(0,255,255),1);


			i = cvRound((m_sVehicleStateToNet.f_slamcorrected_north - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j = cvRound((m_sVehicleStateToNet.f_slamcorrected_east - slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			state_struct sensor_pose;
			sensor_pose.position.x = m_sVehicleStateToNet.f_slamcorrected_east + VEHICLELEN_M * cos(m_sVehicleStateToNet.f_slamcorrected_theta);
			sensor_pose.position.y = m_sVehicleStateToNet.f_slamcorrected_north + VEHICLELEN_M * sin(m_sVehicleStateToNet.f_slamcorrected_theta);
			i1 = cvRound((sensor_pose.position.y-slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j1 = cvRound((sensor_pose.position.x-slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			line(display_src,cvPoint(j ,i),cvPoint(j1 ,i1),Scalar(0,0,255),1);
	        
	        i = cvRound(( y_minrange - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			i1 = cvRound(( y_maxrange - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j = cvRound(( x_minrange - slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			j1 = cvRound((x_maxrange - slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			rectangle(display_src,cvPoint(j, i),cvPoint(j1, i1),Scalar(0,0,255), 2);


			i = cvRound(( m_sVehicleStateToNet.f_slamcorrected_north - 10.0 - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			i1 = cvRound(( m_sVehicleStateToNet.f_slamcorrected_north + 10.0 - slammap_center_pos.y)/globalmapresolution) + (slamogmsize-1)/2 ;
			j = cvRound(( m_sVehicleStateToNet.f_slamcorrected_east - 10.0 - slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			j1 = cvRound((m_sVehicleStateToNet.f_slamcorrected_east + 10.0  - slammap_center_pos.x) /globalmapresolution) + (slamogmsize-1)/2 ;
			rectangle(display_src,cvPoint(j, i),cvPoint(j1, i1),Scalar(0,0,255), 1); 

        }
        //unsigned char* ptr1;
        
        for(int i = 0 ; i < slamogmsize; i++)
        {
          //  ptr1 = display_src.ptr<unsigned char>(i);
            unsigned char* ptr = display_src.ptr<unsigned char>(i);
            for(int j = 0; j <slamogmsize; j ++)
            {
                double val = prob_global_frame[i*slamogmsize+j];
                unsigned char t = (unsigned char)((1- val)*255);
                if(val > UNKNOWNPROB)
                {
            //        unsigned char* ptr=display_src.ptr<unsigned char>((slamogmsize-1-i)*display_src.step);
            
                    ptr[3*j]=0;
                    ptr[3*j+1] = 255;
                    ptr[3*j+2] = 0; 
                }
            }

        }
        
     //flip(display_src,display_src,0);
     imshow("test", display_src);

    //imshow("stest",test_src);
        waitKey(1);
    }
    
    
}

void pfSlam::publishLoop(double tranform_publish_period)
{
    if(tranform_publish_period == 0)
        return;
    ros::Rate r(1.0/ tranform_publish_period);
    while(ros::ok())
    {
        publishTransform();
        r.sleep();
    }
}


void pfSlam::startLiveSlam()
{
    // publish the ogm map? 
    sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map",1, true);

    // this is a service ?  same with the advertiser? 
    ss_ = node_.advertiseService("dynamic_map",&pfSlam::mapCallback, this);
   
    // figure it out how to use message filters to subscriber a certain topic 
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_,"/vrep/front_scan", 5);

    //get the laser lacation relative to odometry frame 
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);

    //bind the laser callback function to scan messages
    scan_filter_->registerCallback(boost::bind(&pfSlam::laserCallback, this, _1));

    //open a thread to publish the tranform 
    transform_thread_ = new boost::thread(boost::bind(&pfSlam::publishLoop, this, transform_publish_period_));

    
}

