
#pragma once
#include <vector>
#include "windows.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdint.h>


using namespace std;

#define pi 3.14159265f

//选择平台
//#define TOYOTAPLATFORM
#define BYDPLATFORM

//是否使用vrep仿真
#define VREPSIMULATION

//是否仿真，实际跑车必须注释掉
#define SIMULATION

//是否保存视频
//#define SAVEVIDEO

//导航信息是否准确。差别在于drpos是用惯导还是用里程计。dr作用：1用于跟踪部分的路网全局局部相互转换用drpos（注意仅限于短时间内切换）。
//slam定位初始化用intergration ，后面更新用dr
//#define POSITIONISACCURATE

//前轮偏角范围
#define STEERINGANGLELIMIT 25.0f

////是否使用鲁棒性因素判断，可能影响通过性
#define USEROBUSTNESS

//是否使用毫米波
#define ENABLERADAR

//能够进行elegantdis判断的最大距离
#define MAXDETECTIONDIS 40.0f

//换道宽度和车道内平移宽度
#define LANEOFFSETVALUE 4.0f
#define LANE_IN_OFFSETVALUE	1.0f

//超车计数器
#define con_safe_counter_threshold 7

//粒子滤波数
#define SAMPLENO 2000

//#define USE_TOPOLOGICAL_MAP


#ifdef TOYOTAPLATFORM
#define DEC							-1.5f
#define ACC							1.0f
#define LFR	2.85f	//前后轴距
#define VEHICLEWIDTH_M	2.0f
#define VEHICLELEN_M	4.0f //车头到后轴
#define NAVI_DIS_TO_BODY 1.0f
#endif

#ifdef BYDPLATFORM
#define DEC							-1.5f 
#define ACC							1.5f
#define LFR	2.66f	//前后轴距
#define VEHICLEWIDTH_M	2.0f
#define MIN_DIST_FOROVERTAKING	10.0
#define DECISIONMAKING_HESITATE_TIME 2.0f

#ifdef VREPSIMULATION
#define VEHICLELEN_M	3.6f //车头到后轴
#else
#define VEHICLELEN_M	4.0f //车头到后轴
#endif
#define NAVI_DIS_TO_BODY 1.0f
#endif

#define MAXLATERALACC				1.0f//最大侧向加速度
 
#define INVALIDVALUE	1000.0f
#define EPSILON_ERROR	0.000001f

//定时器周期
#define	STATUSDISPLAYINTERVAL	500
#define	SYNCINTERVAL	1000

#ifdef VREPSIMULATION//test
#define	PLANINTERVAL	100 
#define CONTROLINTERVAL 200
#else
#define	PLANINTERVAL	100 
#define CONTROLINTERVAL 50
#endif

#define COLORRED	cvScalar(0,0,255)
#define COLORBLUE	cvScalar(255,0,0)
#define COLORGREEN	cvScalar(0,255,0)
#define	COLORWHITE	cvScalar(255,255,255)
#define COLORCYAN	cvScalar(255,255,0)
#define COLORYELLOW	cvScalar(0,255,255)
#define COLORPINK	cvScalar(255,0,255)
#define	COLORGREY	cvScalar(122,122,122)
#define COLORORANGE cvScalar(0,128,255)
#define COLORPURPLE cvScalar(255,0,128)
#define	COLORDARKGREY	cvScalar(188,188,188)
#define	COLORLIGHTGREY	cvScalar(100,100,100)
#define	COLORBLACK	cvScalar(0,0,0)

/////timer
#define STATUSDISPLAYTIMER					2
#define VELCONTROLTIMER						11
#define	TESTDRIVINGTIMER					12
#define	CITYDRIVING							22
#define	SIMCITYDRIVING						23
#define	POSITIONINROADMAP					24
#define	TOPOLOGICALMAPDISPLAY				241
#define AUTOPARKINGTIMER					26
#define EVERY50MILLISECOND		27
#define EVERYSECOND		28
#define EVERY100MILLISECOND		29

#define MAXSPEED		15.0f //10m/s

/************ip和端口定义************/
#define	LANEPORT_LISTEN			5003	//LANE 车道线
#define	NAVIGATIONPORT_LISTEN	5005	//NAVIGATION 监听
#define VELODYNEPORT_LISTEN		5004	//接收velodyne端口
#define ECU_PORT_LISTEN			9002	

#define	NAVIGATIONPORT			5012	//NAVIGATION 主发送
#define	LANEPORT				5042	//LANE 车道线
#define VELODYNEPORT_RIGIDOGM	5052	//VELODYNE 数据来源发送端口
#define ECU_PORT				8002	//NPort 发送数据的端口ECU

#define SENDCMDPORT				5002	//发送控制命令的本地端口

#ifdef VREPSIMULATION
#define IP_ADD_VEHICLE_CONTROL	"127.0.0.1"//"10.1.34.100"//nport--单片机ip
#else
#define IP_ADD_VEHICLE_CONTROL	"192.168.0.254"//nport--单片机ip
#endif

#define NPORT_PORTECU			8002	//nport的ECU端口

#define VREPPORT_LISTEN			9000
#define VREPPORT				9500

//激光网格
#define MAXPROB				0.9f//12.0
#define MINPROB				0.3f//2.0
#define UNKNOWNPROB			0.5f//4.0

#define PROB_OCC			0.7f//0.5
#define PROB_POSSIBLEOCC	0.6f//0.5
#define PROB_UNKNOWN		0.5f//0.0
#define PROB_EMP			0.3f//-0.5

#define OGM_PASS			0
#define OGM_UNKNOWN			1
#define OGM_POSSIBLENOPASS	2
#define OGM_NOPASS			3



/****激光接收*****/
#define SICKPTNUM	761
#define	SICKANGLERESOLUTION	0.25f
#define	SICKANGLERESOLUTIONMINUSONE	4
#define SICKMAXDATANUM		5000
#define STARTANGLE -5 //相对于车辆航向方向 逆时针为正，顺时针为负。
#define ENDANGLE 185




/***********全局地图常量*************/
#define MAP_WIDTH_CELL 600
#define	MAP_HEIGHT_CELL	600


/**********OGM常量******************/
#define OGMDISPLAYZOOM	1
#define OGMRESOLUTION	0.2f
#define OGMWIDTH_M		60	//OGM横轴长度m
#define OGMHEIGHT_M		110	//OGM纵轴长度m
#define	OGMWIDTH_CELL	cvRound(OGMWIDTH_M / OGMRESOLUTION + 1)
#define	OGMHEIGHT_CELL	cvRound(OGMHEIGHT_M / OGMRESOLUTION + 1)
#define VEHICLEPOSINOGM_Y_M 20//车后轴到OGM底边的距离
#define VEHICLEPOSINOGM_X_M 30//车后轴在OGM局部坐标系中的横坐标（m）
#define VEHICLEPOSINOGM_X_CELL cvRound(VEHICLEPOSINOGM_X_M/OGMRESOLUTION)
#define VEHICLELENGTHINCELL	cvRound(VEHICLELEN_M/OGMRESOLUTION)
#define	OUTTERINFLATEDHEIGHT_CELL		VEHICLELENGTHINCELL//CELL 5m
#define	HALFOUTTERINFLATEDWIDTH_CELL		cvRound(VEHICLEWIDTH_M/2/OGMRESOLUTION) //CELL 



//类型定义
//根据路网提供的信息，可以进行一系列细分
enum VirtualLaneDecisionMakingMode{
	KeepCurrentLane,
	Change2LeftLane,
	Change2RightLane,

	Overtake2LeftLane,
	Overtake2RightLane,

	Continue2Change,
	Fail2Change
};

enum VirtualLaneDrivingMode{
	LaneKeeping,
	LaneChangingPreparing,
	LaneChanging,
	OvertakingPreparing,
	Overtaking
};

enum PLANNERMODE{
	VIRTUALLANEPLANNER,
	FREELOCALPLANNER,
	SEARCHBASEDPLANNER
};

enum DRIVEMODE
{
	DRIVEMODE_UNKNOWN,
	DRIVEMODE_LANE,//车道内行驶
	DRIVEMODE_ROADMAP,//自由区域跟随路网行驶
	DRIVEMODE_FIELDMETHOD//纯环境感知行驶
};

enum ASTARPLANFOLLOWSTATE
{
	AStarPlanandFollowWaitForTarget,
	AStarPlanandFollowStopandWaitForTarget,
	AStarPlanandFollowSTARTPLAN,
	AStarPlanandFollowWAITFORPATH,
	AStarPlanandFollowPLANFAIL,
	AStarPlanandFollowFOLLOW,
	AStarPlanandFollowFOLLOWHESITATE
};

enum PCCLOUDPLANNERSTATE
{
	PCCLOUDPLANNER3DIDLE,
	PCCLOUDPLANNER3DWAITFORTARGET,
	PCCLOUDPLANNER3DLEFTBUTTONDOWN,
	PCCLOUDPLANNER3DTARGETGIVEN,
	PCCLOUDPLANNER3DINPLANNING,
	PCCLOUDPLANNER3DPLANFAIL,
	PCCLOUDPLANNER3DPLANSUCCESS
};

enum ASTARFOLLOWINGSTATE
{
	ASTARFOLLOWINGIDLE,
	ASTARFOLLOWINGNOGLOBALPATH,
	ASTARFOLLOWINGSUCCESS,
	ASTARFOLLOWINGFAIL,
	ASTARFOLLOWINGDANGEOUS,
	ASTARFOLLOWINGFINISH
};


//来自路网信息
enum ROADTYPE
{
	STARTUP,
	INTERSECTIONENTER,
	INTERSECTIONEXIT,
	WAYPOINT,
	PARKINGENTER,
	PARKINGEXIT,
	PARKINGSPOT,
	FINISH,
	FIELD,
	ROADMAP,
	UTURNENTER,
	UTURNEXIT,
	LANE
};

//side
enum SIDETYPE
{
	CENTER,
	LEFT,
	RIGHT
};

enum
{
	NO,
	YES
};

enum STOPREASONTYPE
{
	NOSTOP,
	WAITFORSTART,
	NOATTACHMAP,
	OGMINVALID,
	FINISH_STOP,
	ESTOP,
	REDLIGHT,
	PEDESTRIAN,
	LOCALPLANFAIL
};


#define NOSLOW 0x0000
#define ROADMAP_SLOW 0x0001
#define HEADINGOBS_SLOW 0x0002
#define UNSTABLE_SLOW 0x0004
#define ATTRI_SLOW 0x0008
#define SIDE_SLOW 0x0010
#define CURVE_SLOW 0x0020
#define BANDWIDTH_SLOW 0x0040
#define ESTOP_SLOW 0x0080
#define FONS_SLOW 0x0100

struct pose
{
	float x;
	float y;
	double weight;
	float theta;
};


union _uintbyte
{
	uint16_t _uint;
	uint8_t	_uchar[2];
};

union _intbyte
{
	int16_t  _int;
	char	_char[2];
};

/**************结构体***************/
struct pos
{
	float x;
	float y;
};

struct pos_int
{
	int x;
	int y;
};

struct state_struct_simple
{
	pos position;
	float s;//弧长
	float theta;//弧度，车头与正东方向夹角，heading=pi/2-theta; heading_g=heading_l+heading_v
	float prob_x;
	float prob_y;
	float steering_angle;
};

struct state_struct
{
	pos position;
	float s;//弧长
	float theta;//弧度，车头与正东方向夹角，heading=pi/2-theta; heading_g=heading_l+heading_v
	bool forward;//前进或者后退

	float steering_angle;
	float radius;

	//位置可信度
	float prob_x;
	float prob_y;

	//来自路网的属性
	int index;
	char type;
	char side;
	float maxvel;

	//int recovery;
	//int reconstruct;
	//int pedestrian;//pedestrian
	//int waitcounter;//waitcounter//单位100ms

	float range;
	char redlightenable;
};

//struct  state_struct
//{
//	state_struct state;
//	CvKalman* kalman;
//};

struct TrajPoint
{
	float x;//
	float y;//	
	float theta;//车头与正东方向夹角，而坐标系之间的夹角应该是车头与正北的夹角（局部坐标系中与横轴正向夹角，顺时针为正）
	float s;//到零点的弧长
	float ref_s;//对应的ref的弧长
	float k;//曲率，对应前轮偏角
	float steeringangle;//该路径点的转向角

	float maxvel_by_radius;
	float maxvel_by_backstepping;
	float maxvel;//两个trajpoint之间的最大允许速度

	float vel;//
	float t;//到零点的时间

	float prob_x;
	float prob_y;
};

struct Trajectory
{
	vector<TrajPoint> Traj_Points;//车体坐标局部路径
	vector<TrajPoint> Traj_Points_global;//大地坐标局部路径

	int id;
	float lateral_offset;//用于局部路径时，表示与期望路径的关系
	float width;

	//属性
	float s;//路径长度
	float s_afterturning;

	float cost;

	float max_vel;
	float terminalvel;
	bool truncated;
	int dangerour_millimeter_obj_index;

	float safety_dis;
	float elegant_distance;//elegant_distance是需要进行避障的距离，与速度有关。障碍物小于elegant_distance时只需要沿路网前进。在障碍物距离小于elegant_distance之前，
	//必须能够造成车辆减速。只有障碍物距离减小大于车辆减速造成距离小于elegant_distance，才会造成换道。此外，最小的elegant_distance应该能够保证车辆在静止时还有
	//足够的转向空间
	//备注：生成轨迹的长度必须必elegant长，至少保证车辆能够在当前速度下加速超过5m/s，但是从计算量考虑，又不能过长。

	bool selectable;

	//轨迹对应的控制量
	float steeringangle;
	float vel;
	bool use_directcommand;//是直接使用trajectory的控制量还是要进行局部路径跟踪
};

struct virtuallanestate_struct
{
	bool selectable;//车道可选，不可选可能由于车道线虚实或者路沿占据决定
	UINT clearance;//车道线安全情况，0 完全安全， 1 有障碍，2完全挡住
	float v;
	float max_safety_range;//局部规划的最长距离（车道内，不靠边）
};

typedef struct struct_VehicleStateToNet			// 车辆状态
{
	float fPos_east;					// 车辆东向位置(Unit:m)	绝对的GPS坐标，定时初始化integrated位置，用于全局定位，更新周期1s
	float fPos_north;					// 车辆北向位置(Unit:m)	

	float fDRPos_E;					// 航迹推算位置东向(m) 绝对坐标系下的DR结果（从起点起），用于生成全局期望路径路径跟踪控制
	float fDRPos_N;					// 航迹推算位置北向(m)

	float fold_Pos_east;				//记录上次gps定位结果			
	float fold_Pos_north;	
	float fold_DRPos_E;				//记录上次DR定位结果
	float fold_DRPos_N;

	float f_integrated_east;			//用于车辆在全局路径中定位，并从roadmap中确定全局期望路径的起点，更新周期100ms。
	float f_integrated_north;



	/*******地图匹配相关定位结果，不用于规划控制，只用于地图更新********/
	float f_slamcorrected_theta;				
	float f_slamcorrected_east;
	float f_slamcorrected_north;

	///***根据里程计和转弯半径估算的位置，仅作显示用***/
	float Odometer_pre;
	float Odometer_theta;
	float Odometer_x;
	float Odometer_y;


	/**********状态变量*********/
	UCHAR FONSValid ;

	float fForwardVel;					// 车辆纵向速度(Unit:m/s)		
	float fDeForwardVel;
	float fFLRWheelAverAngle;			// 名义前轮偏角，对应电机或方向盘的角度(Unit:°)
	float fHeading;						// 车辆真北航向角(Unit:弧度)
	float fTheta;						//车辆到正东的夹角(Unit:弧度)
	float fOdometer;
	float fRadius;

	unsigned char f_shift;				//档位 0无效1P2R3N4N5D6M7S8+9-
	unsigned char f_shift1;				//具体档位
	unsigned char f_estop;				//紧急制动
	unsigned char f_leftlamp;			//左转向灯
	unsigned char f_rightlamp;			//右转向灯
	bool lateralctrl_enabled;
	bool longitutdectrl_enabled;
	bool brake_enabled;

	DWORD	lTimeStamp;			// 时间戳(Unit:ms)

	//************    BTV AND MSH *******************//
	//***********************************************//
	//***********************************************//

	uint8_t autodrive_status;   //add
	uint8_t brake_pedal_signal;  //add
	uint8_t switch_signal;  //add


	float pressure_back;         //后油路压力
	float petral_pressure;       //制动踏板压力 

	int throtle_feedback;
	uint8_t steerRx_err;
	uint8_t steerTx_err;
	uint8_t brakeRx_err;
	uint8_t brakeTx_err;
	uint8_t PC_Tx_err;

	uint8_t poweron_status;
	uint8_t start_status;
	uint8_t warning_status;
	uint8_t bugle_status;

	uint8_t light_far;
	uint8_t light_near;

	uint8_t Estop_enabled;

}SVehicleStateToNet,*PSVehicleStateToNet;

typedef struct struct_DeVehicleStateToNet		// 期望的车辆状态
{
	unsigned char shift;
	float fDeForwardVel;				// 期望车辆纵向速度(Unit:m/s)		
	float fDeFLRWheelAverAngle;			// 期望车辆左右前轮平均偏角(Unit:°)
	float fDeHeading;					// 期望车辆真北航向角(Unit:°)
	bool lateralctrl_enabled;
	bool longitutdectrl_enabled;
	bool leftlamp_turnedon;
	bool rightlamp_turnedon;
	unsigned int	lTimeStamp;			// 时间戳(Unit:ms)


	float kp , ki , kd ;
	float slope_gas , slope_brake;

	float u_gas;//油门初始值
	float u_brake;//制动初始值

	uint16_t throttle_upper_threshold_low;
	uint16_t throttle_upper_threshold_high;
	uint16_t throttle_actual_threshold;
	uint16_t throttle_init;

	uint16_t brake_upper_threshold_high;
	uint16_t brake_upper_threshold_low;
	uint16_t brake_init;

	uint16_t throttle;
	uint16_t brake;

	uint16_t e_stop_limit;

	int steering_ctrl;

	float delta_u;
	float u;

	//*********   BTV AND MSH ************//
	//************** add *****************//
	//************************************//

	uint8_t auto_enable;
	uint8_t poweron_enable;
	uint8_t engineon_enable;
	uint8_t warning_enable;
	uint8_t bugle_enable;
	uint8_t lightn_enable;
	uint8_t lightf_enable;
	uint8_t shift_enable;


}SDeVehicleStateToNet,*PSDeVehicleStateToNet;


struct Straightline
{
	float x1;
	float y1;
	float x2;
	float y2;
	float theta;
	char continuous;
	char color;
	float len;
};

struct _StraightLaneStruct
{	
	//x1,x2,y1,y2: mm
	float x1;
	float y1;
	float x2;
	float y2;
	float x3;
	float y3;
	float x4;
	float y4;
	char rightcon;//0没有//1间断//2连续
	char leftcon;//0没有//1间断//2连续
	int lanestatus;//车道数
	int prev_lanestatus;
	unsigned int	lTimeStamp;
};

struct _OGMDataStruct
{
	unsigned char* m_OccupiedMap;
	unsigned int	lTimeStamp;
};

struct _RNDFInfoStruct
{
	//其中都用的弧度表示航向
	float distointer;// 到下一个路口节点的距离
	//float distoway;// 到下一个路口节点的距离
	float distonextnode;
	state_struct prev_node_state;//当前节点，用于判断路点属性判断速度
	state_struct next_node_state;//下一节点的位置和期望航向，注意对于路口出点，航向是点的航向，而不是路的，对于路口入点，航向是路的
	state_struct third_node_state;
	state_struct next_intersection_state;
};

struct trajsegmentstruct
{
	int startindex;
	int endindex;
	float width;
	int id;
};

struct segmentstruct
{
	int startindex;
	int endindex;
	float anglerange;
	int id;
	float dis;

	float gap;
	uint8_t feasible;
	char outlet;
	int targetindex;
	float score;
};

struct points2D
{
	float distance[SICKPTNUM];
	float achievable_distance[SICKPTNUM];
	float tmpdistance[SICKPTNUM];
	float intensity[SICKPTNUM];  //20130305添加 保存回波强度
	int id[SICKPTNUM];
	segmentstruct segment[SICKPTNUM];
	float timestamp;
};

struct moving_object_millimeter
{
	//原始数据
	float range;
	int angle;
	float v;
	float x;
	float y;
	float vx;
	float vy;
	bool valid;//测量结果有效，条件是range>2

	////kalman滤波结果
	//CvKalman* kalman;
	//int tracker_lost_count;
	//int track_count;
	//bool tracked;

	//float x_filtered;
	//float y_filtered;
	//float vx_filtered;
	//float vy_filtered;
};
