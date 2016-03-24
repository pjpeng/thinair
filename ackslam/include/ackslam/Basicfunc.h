#pragma once//防止重定义
#include "Notation.h"
#include <vector>
#include "windows.h"
using namespace std;

struct Straightline;

float unify_theta(float theta);

bool IsCellInCone(double x, double y, double theta, 
				  double x1, double y1, int forward_laser_index1,
				  int* laserindex, double *dis, double *deltaphi);

void CalcCenterLine(Straightline* line,_StraightLaneStruct* m_StraightLine);
vector<pos_int> calculatefootfprint(TrajPoint *traj, int width);//¼ÆËãogm×ø±êÏµÏÂµÄfootprintÕ¼¾Ý×ø±ê

unsigned char PosObsCost(pos_int pos, _OGMDataStruct* OGMData_ptr);

void Cal_Point_Dis_to_Line(float pointx, float pointy, float linex1, float liney1, float linex2, float liney2, float *dis, float *u);

float unifytheta(float theta);//½«theta×ª»¯Îª0µ½2piÖ®¼ä

void CalcMediumTermialState(state_struct *terminalstate, float theta);

state_struct Convert_State_Local_to_Global(state_struct localstate, state_struct vehicle_state);//ÔË¶¯×´Ì¬´Ó¾Ö²¿×ªµ½È«¾Ö

state_struct Convert_State_Global_to_Local(state_struct globalstate, state_struct vehicle_state);//ÔË¶¯×´Ì¬´Ó¾Ö²¿×ªµ½È«¾Ö


state_struct_simple Convert_State_Local_to_Global(state_struct_simple localstate, state_struct_simple vehicle_state);//ÔË¶¯×´Ì¬´Ó¾Ö²¿×ªµ½È«¾Ö

state_struct_simple Convert_State_Global_to_Local(state_struct_simple globalstate, state_struct_simple vehicle_state);//ÔË¶¯×´Ì¬´Ó¾Ö²¿×ªµ½È«¾Ö

//¸ù¾Ýµ½refµÄÏà¶ÔÎ»ÖÃ¼ÆËãÔÚogmÖÐµÄ×ø±ê£¬refÊÇÔÚogmÖÐµÄworld×ø±ê
void Convert_World_to_OGM(float x, float y, int * x_ogm, int * y_ogm, float ref_x, float ref_y, float resolution);//³µÌå×ø±êÏµ×ªµ½OGM×ø±êÏµ

//¸ù¾ÝogmÖÐµÄ×ø±ê¼ÆËãµ½refµÄÏà¶ÔÎ»ÖÃ£¬refÊÇÔÚogmÖÐµÄworld×ø±ê
void Convert_OGM_to_World(int x, int y , float* x_world, float* y_world, float ref_x, float ref_y, float resolution);//OGM×ø±êÏµ×ªµ½³µÌå×ø±êÏµ

//¸ù¾Ýµ½refµÄÏà¶ÔÎ»ÖÃ¼ÆËãÔÚÍ¼ÏñÖÐµÄ×ø±ê
void Convert_World_to_IMG(float x, float y, int * x_img, int * y_img, float ref_x, float ref_y, int zoom, int imgheight, float resolution);//³µÌå×ø±êÏµ×ªµ½Í¼Ïñ×ø±êÏµ

//¸ù¾ÝÍ¼ÏñÖÐµÄ×ø±ê¼ÆËãµ½refµÄÏà¶ÔÎ»ÖÃ
void Convert_IMG_to_World(int x, int y , float* x_world, float* y_world, float ref_x, float ref_y, int zoom, int imgheight, float resolution);//Í¼Ïñ×ø±êÏµ×ªµ½³µÌå×ø±êÏµ


float purepursuit_bydeltax(float deltax, float Ld);//purepursuit

float stanley(float deltatheta, float deviation, float k, float v);

float get_deltax_by_steering(float steering, float Ld);

state_struct Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0);

void back_local_planner(Trajectory * traj , float max_vel , float vel_init, float dec);//·´ÏòËÙ¶È¼ÆËã

void local_vel_plan(Trajectory * traj, float vel_init,  float max_vel, float acc, float dec, bool obstaclefound);

void local_vel_plan_smooth(Trajectory * traj, float acc, float dec);

void Traj_plan(Trajectory  *waypoints);

float New_VelocityControlPID(SVehicleStateToNet* m_sVehicleStateToNet_ptr, SDeVehicleStateToNet* sDeVehicleStateToNet_ptr);

void New_VehicleSpeedControl(SVehicleStateToNet* m_sVehicleStateToNet_ptr, SDeVehicleStateToNet* sDeVehicleStateToNet_ptr);

int convert_steeringangle2ctrlvalue(float steeringangle, float steeringratio_l, float steeringratio_r);

float convert_ctrlvalue2steeringangle(int ctrlvalue , bool direction, float steeringratio_l, float steeringratio_r);

void calculateboundrectange(TrajPoint *traj, float width, float *l_t_x , float *l_t_y , float *l_b_x , float *l_b_y, 
							float *r_t_x , float *r_t_y , float *r_b_x , float *r_b_y );

LARGE_INTEGER GetHighAccurateTimeTick();

LARGE_INTEGER GetHighAccurateTime(LARGE_INTEGER lTime);
