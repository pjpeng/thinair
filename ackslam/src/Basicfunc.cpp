#include <cmath>
#include <ackslam/Basicfunc.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

float unify_theta(float theta)
{
	//+-180Ö®¼ä
	while(theta<= -180 || theta>180)
	{
		if(theta>180) 
			theta=theta-360;
		else if(theta<=-180)
			theta=theta+360;
	}
	return theta;
}


bool IsCellInCone(double x, double y, double theta, 
						   double x1, double y1, int forward_laser_index1, 
						   int* laserindex, double *dis, double *deltaphi)
{
	//x,y,theta pose_now
	//x1,y1 center of the cell in global coordinates
	double r = sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
	*dis = r;

	if(r < EPSILON_ERROR){
		return true;
	}
	else if(r<50.0f){
		double phi = atan2(y1-y, x1-x);//+-pi

		double phi_degree = phi * 180 / pi;

		double theta_degree = theta * 180 / pi;

		double gap_degree = phi_degree - theta_degree;
		gap_degree = unify_theta(gap_degree);

		*deltaphi = gap_degree;
		if(fabs(gap_degree) < 90){
			int index=(int)((forward_laser_index1 + gap_degree)/SICKANGLERESOLUTION);
            //cout<<"index:"<<index<<endl;

			if(index>=0 && index<SICKPTNUM){
				*laserindex=index;
				return true;
			}
			else
				return false;
		}
		else
			return false;
	}
	else
		return false;
}

void CalcCenterLine(Straightline *line,_StraightLaneStruct* m_StraightLine)
{
	line->x1=(m_StraightLine->x1+m_StraightLine->x3)/2;
	line->y1=(m_StraightLine->y1+m_StraightLine->y3)/2;
	line->x2=(m_StraightLine->x2+m_StraightLine->x4)/2;
	line->y2=(m_StraightLine->y2+m_StraightLine->y4)/2;
	float len1 = sqrt(pow(m_StraightLine->x1-m_StraightLine->x2,2) + pow(m_StraightLine->y1-m_StraightLine->y2,2));
	float len2 = sqrt(pow(m_StraightLine->x3-m_StraightLine->x4,2) + pow(m_StraightLine->y3-m_StraightLine->y4,2));
	line->len = MAX(len1,len2);

	//¼ÆËã×îÔ¶´¦µÄµã
	float x1,y1,x2,y2;
	y1=0;
	x1=line->x1+(y1-line->y1)*(line->x2-line->x1)/(line->y2-line->y1);
	y2=line->y2;
	x2=line->x1+(y2-line->y1)*(line->x2-line->x1)/(line->y2-line->y1);

	line->x1=x1;
	line->y1=y1;
	line->x2=x2;
	line->y2=y2;

	line->theta = atan2(line->y2-line->y1,line->x2-line->x1);
	line->theta = unifytheta(line->theta);
}

vector<pos_int> calculatefootfprint(TrajPoint *traj, int width)
{
	//µ¥Î»ÊÇcell
	vector<pos_int> footprint;

	pos tmp_pos;
	tmp_pos.x=traj->x / OGMRESOLUTION;//cell
	tmp_pos.y=traj->y / OGMRESOLUTION;

	{
		int i = 0;
		for(int j=- HALFOUTTERINFLATEDWIDTH_CELL;j<=OUTTERINFLATEDHEIGHT_CELL - HALFOUTTERINFLATEDWIDTH_CELL ;j++)
		{
			pos tmp_pos1;
			tmp_pos1.x=i * sin(traj->theta) + j * cos(traj->theta) + tmp_pos.x;
			tmp_pos1.y=-i * cos(traj->theta) + j * sin(traj->theta) + tmp_pos.y;

			pos_int tmp_pos_int;
			tmp_pos_int.x = cvRound(VEHICLEPOSINOGM_X_M / OGMRESOLUTION + tmp_pos1.x);
			tmp_pos_int.y = cvRound(tmp_pos1.y + VEHICLEPOSINOGM_Y_M / OGMRESOLUTION);
			footprint.push_back(tmp_pos_int);

			if(width != 0)
			{
				pos_int tmp_pos_int1;
				tmp_pos_int1.x = tmp_pos_int.x - width;
				tmp_pos_int1.y = tmp_pos_int.y ;
				footprint.push_back(tmp_pos_int1);

				tmp_pos_int1.x = tmp_pos_int.x + width;
				tmp_pos_int1.y = tmp_pos_int.y ;
				footprint.push_back(tmp_pos_int1);
			}
		}
	}
	return footprint;
}

unsigned char PosObsCost(pos_int pos, _OGMDataStruct* OGMData_ptr)
{
	//return 0;
	if(pos.x >= OGMWIDTH_CELL || pos.x< 0 || pos.y>=OGMHEIGHT_CELL || pos.y<0)
	{
		return 0;
	}
	return (*(OGMData_ptr->m_OccupiedMap+pos.y*OGMWIDTH_CELL+pos.x));//Â·ÑØ»òÕß¼¤¹âµã£¬ÔÚogm×ø±êÄÚ½øÐÐ
}

void Cal_Point_Dis_to_Line(float pointx, float pointy, float linex1, float liney1, float linex2, float liney2, float *dis, float *u)
{
	float length=sqrt(pow(liney1-liney2,2)+pow(linex1-linex2,2));
	*dis=fabs((linex2-linex1)*(liney1-pointy)-(linex1-pointx)*(liney2-liney1))/length;

	*u=((pointx-linex1)*(linex2-linex1)+(pointy-liney1)*(liney2-liney1))/(length*length);
}

float unifytheta(float theta)
{
	float result=theta;
	if(theta<0)		result=theta+2*pi;
	if(theta>=2*pi)		result=theta-2*pi;
	return result;
}

void CalcMediumTermialState(state_struct *terminalstate, float theta)
{
	float deltay,deltax;//,heading;

	if(fabs(terminalstate->position.x) > EPSILON_ERROR)
	{
		float slope=(terminalstate->position.y/terminalstate->position.x);
		deltax=terminalstate->position.x/fabs(terminalstate->position.x)*sqrt(50*50/(slope*slope+1));
		deltay=deltax*slope;
	}
	else
	{
		//Ö±ÐÐ
		deltax=0;
		deltay=50;
	}

	terminalstate->position.x=deltax;
	terminalstate->position.y=deltay;

	//¸üÐÂheading
	//if(terminalstate->position.x==0)
	//{
	//	if(terminalstate->position.y>0)
	//		//terminalstate->heading= 0;
	//		terminalstate->theta= pi/2;
	//	else
	//		//terminalstate->heading= pi;
	//		terminalstate->theta=pi*3/2;
	//}
	//else if(terminalstate->position.x>0)
	//	//terminalstate->heading= pi/2-atan((terminalstate->position.y)/(terminalstate->position.x));
	//	terminalstate->theta=atan(terminalstate->position.y/terminalstate->position.x);
	//else
	//	//terminalstate->heading= pi/2-(pi+atan((terminalstate->position .y)/(terminalstate->position.x)));
	//	terminalstate->theta= pi+atan(terminalstate->position.y/terminalstate->position.x);
	terminalstate->theta=theta;

	terminalstate->theta=unifytheta(terminalstate->theta);
}



state_struct Convert_State_Local_to_Global(state_struct localstate, state_struct vehicle_state)
{
	float theta=vehicle_state.theta;

	float newx,newy;
	newx=vehicle_state.position.x+sin(theta)*localstate.position.x+cos(theta)*localstate.position.y;
	newy=vehicle_state.position.y-cos(theta)*localstate.position.x+sin(theta)*localstate.position.y;

	state_struct result;
	result.position.x=newx;
	result.position.y=newy;
	result.theta=localstate.theta+vehicle_state.theta-pi/2;
	result.theta=unifytheta(result.theta);

	result.s=localstate.s;
	result.forward = localstate.forward;

	result.steering_angle=localstate.steering_angle;//Ä¿Ç°Ã»ÓÐ
	return result;
}


state_struct Convert_State_Global_to_Local(state_struct globalstate, state_struct vehicle_state)
{
	float deltax,deltay;
	deltax=globalstate.position.x-vehicle_state.position.x;
	deltay=globalstate.position.y-vehicle_state.position.y;

	float theta=vehicle_state.theta;

	float newx,newy;
	newx=sin(theta)*deltax-cos(theta)*deltay;
	newy=cos(theta)*deltax+sin(theta)*deltay;
	
	state_struct result;
	result.position.x=newx;
	result.position.y=newy;
	result.theta=pi/2+globalstate.theta-vehicle_state.theta;
	result.theta=unifytheta(result.theta);
	
	result.s=globalstate.s;
	result.forward = globalstate.forward;

	result.steering_angle=globalstate.steering_angle;//Ä¿Ç°Ã»ÓÐ
	return result;
}


state_struct_simple Convert_State_Local_to_Global(state_struct_simple localstate, state_struct_simple vehicle_state)
{
	float theta=vehicle_state.theta;

	float newx,newy;
	newx=vehicle_state.position.x+sin(theta)*localstate.position.x+cos(theta)*localstate.position.y;
	newy=vehicle_state.position.y-cos(theta)*localstate.position.x+sin(theta)*localstate.position.y;

	state_struct_simple result;
	result.position.x=newx;
	result.position.y=newy;
	result.theta=localstate.theta+vehicle_state.theta-pi/2;
	result.theta=unifytheta(result.theta);

	result.s=localstate.s;
	
	return result;
}


state_struct_simple Convert_State_Global_to_Local(state_struct_simple globalstate, state_struct_simple vehicle_state)
{
	float deltax,deltay;
	deltax=globalstate.position.x-vehicle_state.position.x;
	deltay=globalstate.position.y-vehicle_state.position.y;

	float theta=vehicle_state.theta;

	float newx,newy;
	newx=sin(theta)*deltax-cos(theta)*deltay;
	newy=cos(theta)*deltax+sin(theta)*deltay;

	state_struct_simple result;
	result.position.x=newx;
	result.position.y=newy;
	result.theta=pi/2+globalstate.theta-vehicle_state.theta;
	result.theta=unifytheta(result.theta);

	result.s=globalstate.s;
	
	return result;
}

/*******×ø±êÏµ×ª»»£¬×¢Òâ¶¼Ó¦ÔÚÊÀ½ç×ø±êÏµÏÂÍê³É£¬±ÜÃâ½Ø¶ÏÎó²î******/
void Convert_World_to_OGM(float x, float y, int * x_ogm, int * y_ogm, float ref_x, float ref_y, float resolution)
{//m to cell
	*x_ogm = cvRound((ref_x + x)/resolution);
	*y_ogm = cvRound((ref_y + y)/resolution);
}

void Convert_OGM_to_World(int x, int y , float* x_world, float* y_world, float ref_x, float ref_y, float resolution)
{//cell to m
	*x_world = x * resolution - ref_x;
	*y_world = y * resolution - ref_y;
}

void Convert_World_to_IMG(float x, float y, int * x_img, int * y_img, float ref_x, float ref_y, int zoom, int imgheight, float resolution)
{//m to cell
	*x_img = cvRound((x + ref_x) * zoom / resolution);
	*y_img = imgheight * zoom - cvRound((y + ref_y) * zoom / resolution) - 1;
}

void Convert_IMG_to_World(int x, int y , float* x_world, float* y_world, float ref_x, float ref_y, int zoom, int imgheight, float resolution)
{//cell to m
	*x_world = x * resolution / zoom - ref_x;
	*y_world = (imgheight * zoom - y - 1) * resolution / zoom - ref_y;
}

float purepursuit_bydeltax(float deltax, float Ld)
{
	//ldÔ¤Ãé¾àÀë
	if(deltax > Ld) deltax = Ld;
	else if(deltax < -Ld) deltax = -Ld;
	float exp_ang= (float)atan2(2*(float)LFR*deltax,(Ld*Ld));
	exp_ang=exp_ang * 180/pi;
	return exp_ang;
}

float stanley(float deltatheta, float deviation, float k, float v){
	//deltathetaÊÇÆÚÍûÂ·¾¶º½ÏòÓë³µÁ¾º½ÏòÆ«²î£¬deviationÊÇ³µÁ¾µ½ÆÚÍûÂ·¾¶¾àÀë
	float exp_ang = 0.0f;

	exp_ang = deltatheta + atan(k * deviation / v);

	exp_ang = exp_ang * 180 / pi;

	return exp_ang;
}

float get_deltax_by_steering(float steering, float Ld)
{
	float steering_radius = steering * pi / 180;
	float deltax = tan(steering_radius) * Ld * Ld / (2 * LFR);

	if(deltax > Ld) deltax = Ld;
	else if(deltax < -Ld) deltax = -Ld;

	return deltax;
}

state_struct Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0)
{
	state_struct result;
	double WGS84_ECCENTRICITY = (double)0.0818192;						  // e=0.0818192
	double WGS84_EQUATORIAL_RADIUS = (double)6378.137;					  // a=6378.137
	double k0 = (double)0.9996;
	
	int Zone = (int)(longitude/6) + 1;									  // ¼ÆËãËùÔÚÇøÓò
	int lonBase = Zone*6 - 3;// ¼ÆËãËùÔÚÇøÓòÖÐÑë×ÓÎçÏß
	
	//ÒÔÏÂ°´ÕÕ¹«Ê½¼ÆËã
	double   vPhi = (double)(1 / sqrt(1-pow(WGS84_ECCENTRICITY * sin(latitude*pi/180.0),2)));
	double	A	 = (double)(( longitude - lonBase )*pi/180.0 * cos(latitude*pi/180.0));		// °´ÕÕ¶«°ëÇòÀ´Ëã
	double	sPhi = (double)((1 - pow(WGS84_ECCENTRICITY,2)/4.0 - 3*pow(WGS84_ECCENTRICITY,4)/64.0 - 5*pow(WGS84_ECCENTRICITY,6)/256.0) * latitude*pi/180.0
					- (3*pow(WGS84_ECCENTRICITY,2)/8.0 + 3*pow(WGS84_ECCENTRICITY,4)/32.0 + 45*pow(WGS84_ECCENTRICITY,6)/1024.0) * sin(2*latitude*pi/180.0)
					+ (15*pow(WGS84_ECCENTRICITY,4)/256.0 + 45*pow(WGS84_ECCENTRICITY,6)/256.0)* sin(4*latitude*pi/180.0)
					- (35*pow(WGS84_ECCENTRICITY,6)/3072.0) * sin(6*latitude*pi/180.0));
	double	T	= (double)(pow(tan(latitude*pi/180.0),2));
	double	C	= (double)((pow(WGS84_ECCENTRICITY,2)/(1 - pow(WGS84_ECCENTRICITY,2))) * pow(cos(latitude*pi/180.0),2));

	result.position.x = (double)((k0*WGS84_EQUATORIAL_RADIUS*vPhi*(A + (1 - T + C)*pow(A,3)/6.0 
				+ (5 - 18*T + pow(T,2))*pow(A,5)/120.0))*1000 - e0);
	result.position.y = (double)((k0*WGS84_EQUATORIAL_RADIUS*(sPhi + vPhi * tan(latitude*pi/180.0)*(pow(A,2)/2 
				+ (5 - T + 9*C + 4*C*C)*pow(A,4)/24.0 + (61 - 58*T + T*T)*pow(A,6)/720.0)))*1000 - n0);
	return result;
	
}

void back_local_planner(Trajectory * traj , float max_vel , float vel_init, float dec){
	if(traj->Traj_Points.size() > 1 ){
		traj->Traj_Points.at(traj->Traj_Points.size()-1).maxvel_by_backstepping = 
			traj->Traj_Points.at(traj->Traj_Points.size()-1).vel;
		if(traj->Traj_Points.at(traj->Traj_Points.size()-1).maxvel_by_backstepping > max_vel)
			traj->Traj_Points.at(traj->Traj_Points.size()-1).maxvel_by_backstepping = max_vel;

		float terminalvel = traj->Traj_Points.at(traj->Traj_Points.size()-1).maxvel_by_backstepping;
		int terminalindex = traj->Traj_Points.size()-1;
		//È·¶¨safety_disÄÚµÄµÚÒ»¸öµã
		for(int i=traj->Traj_Points.size()-1; i>=0; i--){
			if(traj->Traj_Points.at(traj->Traj_Points.size()-1).s - traj->Traj_Points.at(i).s < traj->safety_dis){
				terminalindex=i;
				traj->Traj_Points.at(i).maxvel_by_backstepping = traj->Traj_Points.at(traj->Traj_Points.size()-1).maxvel_by_backstepping;
			}
			else
				break;
		}

		//ÓÃdecµ¹ÍË¼ÆËã
		for(int i = terminalindex - 1 ; i >= 0 ; i--){// v2*v2 = 2 * dec * s +¡¡v1*v1
			float deltas = traj->Traj_Points.at(i+1).s - traj->Traj_Points.at(i).s;
			traj->Traj_Points.at(i).maxvel_by_backstepping = 
				sqrt(fabs(traj->Traj_Points.at(i+1).maxvel_by_backstepping * traj->Traj_Points.at(i+1).maxvel_by_backstepping - 2 * dec * deltas));	
		}
	}
}

void local_vel_plan(Trajectory * traj, float vel_init, float max_vel, float acc, float dec, bool obstaclefound)
{	
	//max_velÊÇÒ»¸ö²Î¿¼Öµ£¬Èç¹ûvel_init>max_vel£¬¹æ»®Æ÷»á³¢ÊÔÓÃÕý³£¼õËÙ¶È½«³µËÙ½µµ½max_vel£¬Èç¹û¾àÀë²»¹»Ôò»á½«¼õËÙ¶È¼Ó´ó¡£
	//ÓÉÓÚËÙ¶È¹æ»®Íê³Éºó±ãÒªÖ´ÐÐ£¬Òò´ËËÙ¶È¹æ»®²»ÔÙÓ°Ïìfeasible.
	
	float S1,S2,S3,S4,S5;
	
	//ËÙ¶È¹æ»®
	if(vel_init >= max_vel) vel_init = max_vel;

	traj->Traj_Points.at(0).vel=vel_init;//³õÊ¼µãËÙ¶È¸³Öµ

	if(traj->Traj_Points.at(traj->Traj_Points.size()-1).vel>max_vel)
		traj->Traj_Points.at(traj->Traj_Points.size()-1).vel=max_vel;

	float terminalvel=traj->Traj_Points.at(traj->Traj_Points.size()-1).vel;
	int terminalindex=traj->Traj_Points.size()-1;

	//Ç°·½Èç¹ûÓÐÕÏ°­Îï£¬È·±£ÔÚ¾àÀëÕÏ°­Îï2*velµÄ¾àÀëËÙ¶ÈÓëÕÏ°­Îï±£³ÖÒ»ÖÂ£¬°²È«¾àÀëÖÁÉÙÎª3m
	{
		for(int i=traj->Traj_Points.size()-1;i>=0;i--){
			if(traj->Traj_Points.at(traj->Traj_Points.size()-1).s-traj->Traj_Points.at(i).s<traj->safety_dis){
				terminalindex=i;
				traj->Traj_Points.at(i).vel=traj->Traj_Points.at(traj->Traj_Points.size()-1).vel;
			}
			else
				break;
		}
	}

	{
		S3=traj->Traj_Points.at(terminalindex).s-traj->Traj_Points.at(0).s;//ÄÜ¹»ÓÃÀ´½øÐÐËÙ¶È¹æ»®µÄ×Ü³¤£¬È¥µôÇ°ÃæÑÓÊ±ºÍºóÃæµÄ°²È«¾àÀë
		S1=(max_vel*max_vel-vel_init*vel_init)/(2*acc);//¼ÓËÙ¶Î
		S2=(terminalvel*terminalvel-max_vel*max_vel)/(2*dec);//¼õËÙ¶Î
		S4=S1+S2;//¼ÓËÙ¶Î¼Ó¼õËÙ¶Î

		if(S3>S4){
			//ÄÜ¹»µ½´ï×î¸ßËÙ¶È£¬ÖÐ¼äÓÐÔÈËÙ×´Ì¬£¬done
			S2=S3-S2;//¿ªÊ¼¼õËÙµã

			for(int i=1;i<terminalindex;i++){
				if((traj->Traj_Points.at(i).s-traj->Traj_Points.at(0).s)<S1){
					float tmp_vel=(traj->Traj_Points.at(i-1).vel*traj->Traj_Points.at(i-1).vel+2*acc*(traj->Traj_Points.at(i).s-traj->Traj_Points.at(i-1).s));
					if(tmp_vel>0)
						traj->Traj_Points.at(i).vel=sqrt(tmp_vel);
					else
						traj->Traj_Points.at(i).vel=0;
				}
				else if((traj->Traj_Points.at(i).s-traj->Traj_Points.at(0).s)<S2)
					traj->Traj_Points.at(i).vel=max_vel;
				else{
					float tmp_vel=(traj->Traj_Points.at(i-1).vel*traj->Traj_Points.at(i-1).vel+2*dec*(traj->Traj_Points.at(i).s-traj->Traj_Points.at(i-1).s));
					if(tmp_vel>0)
						traj->Traj_Points.at(i).vel=sqrt(tmp_vel);
					else
						traj->Traj_Points.at(i).vel=0;
				}
			}

			for( int i=1;i<terminalindex;i++){//¿¼ÂÇ¶¯Á¦Ñ§ÒòËØ
				if(traj->truncated){
					float maxvel_by_overtaking = fabs(traj->s - traj->Traj_Points.at(i).s - MIN_DIST_FOROVERTAKING) / (2.0f + DECISIONMAKING_HESITATE_TIME);
					traj->Traj_Points.at(i).vel=min(maxvel_by_overtaking, traj->Traj_Points.at(i).vel);
				}
				traj->Traj_Points.at(i).vel=min(traj->Traj_Points.at(i).maxvel_by_radius, traj->Traj_Points.at(i).vel);
				traj->Traj_Points.at(i).vel=min(traj->Traj_Points.at(i).maxvel_by_backstepping,traj->Traj_Points.at(i).vel);
				traj->Traj_Points.at(i).maxvel = min(traj->Traj_Points.at(i).maxvel_by_radius, traj->Traj_Points.at(i).maxvel_by_backstepping);
			}
		}
		else{
			//ÎÞ·¨´ïµ½×î¸ßËÙ¶È£¬ÖÐ¼äÃ»ÓÐÔÈËÙ×´Ì¬£¬È·¶¨S5
			if( ((terminalvel*terminalvel-vel_init*vel_init) / (2 * acc)) >= S3){
				//Ã»ÓÐ¼õËÙ¶Î
				for(int i=1;i<terminalindex;i++)
					traj->Traj_Points.at(i).vel = MIN(terminalvel , sqrt(2 * acc *traj->Traj_Points.at(i).s + vel_init * vel_init));
			}
			else{
				//´æÔÚ¼õËÙ¶Î
				float exp_del=(terminalvel*terminalvel-vel_init*vel_init)/(2*S3);

				if(exp_del>=dec){
					//ÈÔÈ»»áÏÈÕý³£¼ÓËÙºóÕý³£¼õËÙ
					if((2*acc*dec*S3+dec*vel_init*vel_init-acc*terminalvel*terminalvel)/(dec-acc)<0)
						max_vel=0;
					else
						max_vel=sqrt((2*acc*dec*S3+dec*vel_init*vel_init-acc*terminalvel*terminalvel)/(dec-acc));//¼ÆËãÄÜ¹»´ïµ½µÄ×î´óËÙ¶È

					S5=(max_vel*max_vel-vel_init*vel_init)/(2*acc);//S5ÊÇÓÉ¼ÓËÙ±ä³É¼õËÙµÄÎ»ÖÃ£¬²»¿¼ÂÇ¶¯Á¦Ñ§Ô¼ÊøÇé¿öÏÂ

					//i=0£¬×´Ì¬Îª³õÊ¼µã×´Ì¬
					for(int i=1;i<terminalindex;i++){
						float tmp_vel;
						if((traj->Traj_Points.at(i).s-traj->Traj_Points.at(0).s)<S5){
							tmp_vel=(traj->Traj_Points.at(i-1).vel*traj->Traj_Points.at(i-1).vel+2*acc*(traj->Traj_Points.at(i).s-traj->Traj_Points.at(i-1).s));
						}
						else{
							tmp_vel=(traj->Traj_Points.at(i-1).vel*traj->Traj_Points.at(i-1).vel+2*dec*(traj->Traj_Points.at(i).s-traj->Traj_Points.at(i-1).s));
						}
						if(tmp_vel>0)
							traj->Traj_Points.at(i).vel=sqrt(tmp_vel);
						else
							traj->Traj_Points.at(i).vel=0;
					}
				}
				else{
					//Õý³£¼õËÙÒÑ¾­²»×ãÒÔÍ£³µ£¬ÓÃ´ó¼õËÙ¶ÈÍ£³µ
					for(int i=1;i<terminalindex;i++){
						float tmp_vel=(traj->Traj_Points.at(i-1).vel*traj->Traj_Points.at(i-1).vel+2*exp_del*(traj->Traj_Points.at(i).s-traj->Traj_Points.at(i-1).s));
						if(tmp_vel>0)
							traj->Traj_Points.at(i).vel=sqrt(tmp_vel);
						else
							traj->Traj_Points.at(i).vel=0;
					}
				}
			}

			for( int i=1;i<terminalindex;i++){//¿¼ÂÇ¶¯Á¦Ñ§ÒòËØ
				if(traj->truncated){
					float maxvel_by_overtaking = fabs(traj->s - traj->Traj_Points.at(i).s - MIN_DIST_FOROVERTAKING) / (2.0f + DECISIONMAKING_HESITATE_TIME);
					traj->Traj_Points.at(i).vel=min(maxvel_by_overtaking, traj->Traj_Points.at(i).vel);
				}
				traj->Traj_Points.at(i).vel=min(traj->Traj_Points.at(i).maxvel_by_radius, traj->Traj_Points.at(i).vel);
				traj->Traj_Points.at(i).vel=min(traj->Traj_Points.at(i).maxvel_by_backstepping,traj->Traj_Points.at(i).vel);
				traj->Traj_Points.at(i).maxvel = min(traj->Traj_Points.at(i).maxvel_by_radius, traj->Traj_Points.at(i).maxvel_by_backstepping);
			}
		}
	}
}

void local_vel_plan_smooth(Trajectory * traj, float acc, float dec){
	if(traj->Traj_Points.size() > 1){
		for(int i = traj->Traj_Points.size() - 1; i > 0; i--){
			//ÓÃdecµ¹ÍË¼ÆËã
			for(int j = i - 1 ; j >= 0 ; j--){// v2*v2 = 2 * dec * s +¡¡v1*v1
				float deltas = traj->Traj_Points.at(j+1).s - traj->Traj_Points.at(j).s;
				float vel_by_backstepping = 
					sqrt(fabs(traj->Traj_Points.at(j+1).vel * traj->Traj_Points.at(j+1).vel - 2 * dec * deltas));	

				if(traj->Traj_Points.at(j).vel > vel_by_backstepping){
					traj->Traj_Points.at(j).vel = vel_by_backstepping;
					traj->Traj_Points.at(i).maxvel = min(traj->Traj_Points.at(i).maxvel, vel_by_backstepping);
				}
				else
					break;
			}
		}

		for(UINT i = 0; i < traj->Traj_Points.size(); i++){
			//ÓÃaccÕýÏò¼ÆËã
			for(UINT j = i + 1; j < traj->Traj_Points.size(); j++){
				float deltas = traj->Traj_Points.at(j).s - traj->Traj_Points.at(j - 1).s;
				float vel_by_forwardstepping = 
					sqrt(fabs(traj->Traj_Points.at(j-1).vel * traj->Traj_Points.at(j-1).vel + 2 * acc * deltas));	
				
				if(traj->Traj_Points.at(j).vel > vel_by_forwardstepping){
					traj->Traj_Points.at(j).vel = vel_by_forwardstepping;
					traj->Traj_Points.at(i).maxvel = min(traj->Traj_Points.at(i).maxvel, vel_by_forwardstepping);
				}
				else
					break;
			}
		}
	}
}

void Traj_plan(Trajectory *waypoints)
{
	waypoints->Traj_Points.at(0).t=0;

	for(unsigned int t=1;t<waypoints->Traj_Points.size();t++)
	{
		if(waypoints->Traj_Points.at(t-1).vel+waypoints->Traj_Points.at(t).vel >0 )
		{
			float deltat=2*(waypoints->Traj_Points.at(t).s-waypoints->Traj_Points.at(t-1).s)/(waypoints->Traj_Points.at(t-1).vel+waypoints->Traj_Points.at(t).vel);
			waypoints->Traj_Points.at(t).t=waypoints->Traj_Points.at(t-1).t+deltat;
		}
		else
			waypoints->Traj_Points.at(t).t=waypoints->Traj_Points.at(t-1).t;
	}
}

static float ek_0 = 0.0f , ek_1 = 0.0f , ek_2 = 0.0f , lastctrl = 0.0f;

float New_VelocityControlPID(SVehicleStateToNet* m_sVehicleStateToNet_ptr, SDeVehicleStateToNet* sDeVehicleStateToNet_ptr )
{	
	float gas_ctrl_limit , brake_ctrl_limit , brake_ctrl_limit_lower;

	gas_ctrl_limit = (sDeVehicleStateToNet_ptr->throttle_actual_threshold - sDeVehicleStateToNet_ptr->throttle_init ) / sDeVehicleStateToNet_ptr->slope_gas;

	brake_ctrl_limit = (sDeVehicleStateToNet_ptr->brake_upper_threshold_high - sDeVehicleStateToNet_ptr->brake_init ) / sDeVehicleStateToNet_ptr->slope_brake;

	brake_ctrl_limit_lower = (sDeVehicleStateToNet_ptr->brake_upper_threshold_low - sDeVehicleStateToNet_ptr->brake_init ) / sDeVehicleStateToNet_ptr->slope_brake;


	float fCurrentVelcoityValue = m_sVehicleStateToNet_ptr->fForwardVel;

	float fDesireVelcoityValue = sDeVehicleStateToNet_ptr->fDeForwardVel;

	ek_0 = fDesireVelcoityValue - fCurrentVelcoityValue;	//ËÙ¶ÈÆ«²î
	
	sDeVehicleStateToNet_ptr->delta_u = sDeVehicleStateToNet_ptr->kp * ( ek_0 - ek_1) + sDeVehicleStateToNet_ptr->ki * ek_0 + sDeVehicleStateToNet_ptr->kd * (ek_0 - 2*ek_1 + ek_2);	 //PIDÔöÁ¿

	float Uaction = (lastctrl + sDeVehicleStateToNet_ptr->delta_u);
 	
	//UactionµÄÏÞÖÆ£¬¶ÔÓ¦throttleºÍbrakeµÄÉÏÏÞ
	if(Uaction > gas_ctrl_limit)
		Uaction = gas_ctrl_limit;
	else if(Uaction < brake_ctrl_limit)
		Uaction = brake_ctrl_limit;

	ek_2 = ek_1;
	ek_1 = ek_0;
	lastctrl = Uaction; 
	



	/****ÌØÊâ´¦Àí****/
	//¹æ»®ËÙ¶ÈÎªÁã£¬ËÙ¶È½ÏµÍÊ±Êä³öÖÆ¶¯×î´óÖµ£¨½ÏÆ½Ë³µÄ£©
	if(m_sVehicleStateToNet_ptr->fForwardVel < 0.5 && sDeVehicleStateToNet_ptr->fDeForwardVel < 0.1)
	{
		Uaction = brake_ctrl_limit_lower;
		lastctrl = Uaction; 
	}

	//¹æ»®ËÙ¶ÈÎªÁã£¬ËÙ¶È½Ï¸ßÊ±Êä³öÖÆ¶¯×î´óÖµ£¨½ÏÆ½Ë³µÄ£©
	if(m_sVehicleStateToNet_ptr->fForwardVel > 0.5 && sDeVehicleStateToNet_ptr->fDeForwardVel < 0.1)
	{
		Uaction = brake_ctrl_limit;
		lastctrl = Uaction; 
	}

	//test£¬Èç¹ûÆÚÍûÓÐËÙ¶È£¬µ«ÊÇµ±Ç°Ã»ÓÐËÙ¶È£¬Ôò½«u¸ø³É0£¬·Å¿ªÖÆ¶¯Ì¤°å¡£ÓÃÓÚ·ÀÖ¹µÍËÙÐÐÊ»ËÉÖÆ¶¯Ì«Âý
#ifndef VREPSIMULATION
	if(m_sVehicleStateToNet_ptr->fForwardVel < 0.2 && sDeVehicleStateToNet_ptr->fDeForwardVel > 0.3)
	{
		Uaction = 0 ;
		lastctrl = Uaction;
	}
#endif

	//test Èç¹ûÆÚÍûËÙ¶È±Èµ±Ç°ËÙ¶ÈÐ¡³¬¹ý5m/s£¬±£³Ö×î´óÖÆ¶¯Á¿
	if(m_sVehicleStateToNet_ptr->fForwardVel - sDeVehicleStateToNet_ptr->fDeForwardVel > 5.0f)
	{
		Uaction = brake_ctrl_limit;
		lastctrl = Uaction;
	}

	//estopÊ¹ÄÜ£¬×ÝÏò¿ØÖÆÁ¿ÇåÁã
	if(m_sVehicleStateToNet_ptr->f_estop == 1)
	{
		ek_0 = 0.0f ;
		ek_1 = 0.0f ;
		ek_2 = 0.0f ;
		lastctrl = 0.0f;
		Uaction = 0.0f;
	}

#ifdef BYDPLATFORM
	// ±ÈÑÇµÏÈç¹û·¢ÏÖ²»ÊÇ¹æ»®Ôì³ÉµÄÉ²³µ£¬»òÕß×ÝÏò²»Ê¹ÄÜ£¬×ÝÏò¿ØÖÆÁ¿ÇåÁã£¬ÕâÑùÖ»ÒªÈ¡Ïû×ÝÏòÊ¹ÄÜ£¬¾ÍÄÜÊ¹×ÝÏò¿ØÖÆÇåÁã¡£
	//×¢ÒâÕâÀïµÄÊ¹ÄÜÊÇÆÚÍûÊ¹ÄÜ£¬¶ø²»ÊÇÊµ¼Ê×´Ì¬µÄÊ¹ÄÜ£¬·ñÔòÈË¹¤×ÔÖ÷ÇÐ»»Ê±»á³öÏÖ³å»÷¡£
	if( (m_sVehicleStateToNet_ptr->brake_enabled && Uaction > sDeVehicleStateToNet_ptr->u_gas) || (!sDeVehicleStateToNet_ptr->longitutdectrl_enabled))
	{
		ek_0 = 0.0f ;
		ek_1 = 0.0f ;
		ek_2 = 0.0f ;
		lastctrl = 0.0f;
		Uaction = 0.0f;
	}
#endif

	return Uaction;
}

void New_VehicleSpeedControl(SVehicleStateToNet* m_sVehicleStateToNet_ptr, SDeVehicleStateToNet* sDeVehicleStateToNet_ptr)
{
	//Éè¶¨ÓÍÃÅÖÆ¶¯×î´óÖµ
#ifdef TOYOTAPLATFORM
	if(m_sVehicleStateToNet_ptr->fForwardVel < 3.0f)
		sDeVehicleStateToNet_ptr->throttle_actual_threshold = 5000;
	else if(m_sVehicleStateToNet_ptr->fForwardVel < 6.0f)
		sDeVehicleStateToNet_ptr->throttle_actual_threshold = sDeVehicleStateToNet_ptr->throttle_upper_threshold_low;
	else
		sDeVehicleStateToNet_ptr->throttle_actual_threshold = sDeVehicleStateToNet_ptr->throttle_upper_threshold_high;
#endif


#ifdef BYDPLATFORM
	if(m_sVehicleStateToNet_ptr->f_shift == 4)//Ç°½øµ²
	{
		if(m_sVehicleStateToNet_ptr->f_shift1 == 1)//1µµ
			sDeVehicleStateToNet_ptr->throttle_actual_threshold = 55;	
		else if(m_sVehicleStateToNet_ptr->f_shift1 == 2)//2µµ
			sDeVehicleStateToNet_ptr->throttle_actual_threshold = 60;
		else if(m_sVehicleStateToNet_ptr->f_shift1 == 3)//3µµ
		{
			if (m_sVehicleStateToNet_ptr->fForwardVel < 9.0f)
				sDeVehicleStateToNet_ptr->throttle_actual_threshold = 50;
			else if(m_sVehicleStateToNet_ptr->fForwardVel < 11.0f)
				 sDeVehicleStateToNet_ptr->throttle_actual_threshold = 52;
			else 
				sDeVehicleStateToNet_ptr->throttle_actual_threshold = 55;
		}
		else
			sDeVehicleStateToNet_ptr->throttle_actual_threshold = 50;
	}
	else//µ¹µµ
	{
		sDeVehicleStateToNet_ptr->throttle_actual_threshold = 0;
	}	
#endif

	//PID¿ØÖÆ
	sDeVehicleStateToNet_ptr->u = New_VelocityControlPID(m_sVehicleStateToNet_ptr , sDeVehicleStateToNet_ptr);

	//¸ù¾Ýpid½á¹ûÈ·¶¨¿ØÖÆÁ¿
    if(sDeVehicleStateToNet_ptr->u > sDeVehicleStateToNet_ptr->u_gas)//¼ÓËÙ½×¶Î
    {
		sDeVehicleStateToNet_ptr->brake = 0;
		sDeVehicleStateToNet_ptr->throttle = (UINT16)(sDeVehicleStateToNet_ptr->throttle_init + sDeVehicleStateToNet_ptr->slope_gas * sDeVehicleStateToNet_ptr->u);

#ifdef BYDPLATFORM
		//¼ÓËÙÌØÊâ´¦Àí
		if(m_sVehicleStateToNet_ptr->f_shift1 == 2 && m_sVehicleStateToNet_ptr->fForwardVel > 10.0f)
		{
			sDeVehicleStateToNet_ptr->throttle = 40;
			lastctrl = (sDeVehicleStateToNet_ptr->throttle - sDeVehicleStateToNet_ptr->throttle_init) / sDeVehicleStateToNet_ptr->slope_gas ;
		}
	/*	if(m_sVehicleStateToNet_ptr->f_shift1 == 3 && m_sVehicleStateToNet_ptr->fForwardVel > 12.0f)
		{
			sDeVehicleStateToNet_ptr->throttle = 36;
			lastctrl = (sDeVehicleStateToNet_ptr->throttle - sDeVehicleStateToNet_ptr->throttle_init) / sDeVehicleStateToNet_ptr->slope_gas ;
		}*/
#endif

    }
    else if(sDeVehicleStateToNet_ptr->u < sDeVehicleStateToNet_ptr->u_brake)//ÖÆ¶¯½×¶Î
    {
		sDeVehicleStateToNet_ptr->throttle = 0;
		if(1)
			sDeVehicleStateToNet_ptr->brake = (UINT16)(sDeVehicleStateToNet_ptr->brake_init + sDeVehicleStateToNet_ptr->slope_brake * sDeVehicleStateToNet_ptr->u);
		else
		{
			//Ö»ÒªÖÆ¶¯¾Í±£³ÖÔÚ×î´óÖÆ¶¯Á¿
			if(m_sVehicleStateToNet_ptr->fForwardVel > 0.5)
				sDeVehicleStateToNet_ptr->brake = sDeVehicleStateToNet_ptr->brake_upper_threshold_high;
			else
				sDeVehicleStateToNet_ptr->brake = sDeVehicleStateToNet_ptr->brake_upper_threshold_low;
		}
    }
	else//µ¡ËÙ½×¶Î
	{
		sDeVehicleStateToNet_ptr->throttle = 0;
		sDeVehicleStateToNet_ptr->brake = 0;
	}

	//³¢ÊÔËÄÉáÎåÈëµÄ×ÝÏò¿ØÖÆ£¬µ¥Î»Îª100
#ifdef TOYOTAPLATFORM
	sDeVehicleStateToNet_ptr->throttle = sDeVehicleStateToNet_ptr->throttle / 100 * 100;
	sDeVehicleStateToNet_ptr->brake = sDeVehicleStateToNet_ptr->brake / 100 * 100;
#endif


#ifdef BYDPLATFORM
	sDeVehicleStateToNet_ptr->brake = sDeVehicleStateToNet_ptr->brake / 100 * 100;
#endif

}

int convert_steeringangle2ctrlvalue(float steeringangle, float steeringratio_l, float steeringratio_r)
{
	//Ä¿Ç°½öÊ¹ÓÃÒ»¸ö½á¹û
	int steering_ctrl ;
#ifdef TOYOTAPLATFORM
	steering_ctrl = 0;
	if( steeringangle > 0)//×ó×ª
		steering_ctrl = (int)( steeringratio_l * steeringangle);
	else//ÓÒ×ª
		steering_ctrl = (int)(- steeringratio_r * steeringangle);
#endif



#ifdef BYDPLATFORM
	steering_ctrl = 7800;
	if(steeringangle > 0)
		steering_ctrl = (int) (steeringratio_l * steeringangle + 7800);
	else
		steering_ctrl = (int) (steeringratio_r * steeringangle + 7800);
#endif

	return steering_ctrl;
}

float convert_ctrlvalue2steeringangle(int ctrlvalue, bool direction, float steeringratio_l, float steeringratio_r)
{
	float steeringangle=0.0f;
#ifdef TOYOTAPLATFORM
	//float TOYOTA_Steering_ratio_L = 2000 / 16.04 ;
	//float TOYOTA_Steering_ratio_R = 2000 / 16.22 ;

	if(!direction)//×ó×ª
		steeringangle = (float) ctrlvalue / steeringratio_l;
	else//ÓÒ×ª
		steeringangle = (float) - ctrlvalue / steeringratio_r;

#endif


#ifdef BYDPLATFORM
	//float BYD_Steering_ratio_L = 2698 / 13.96 ;
	//float BYD_Steering_ratio_R = 2545 / 13.25 ;

	steeringangle = (float)(ctrlvalue - 7800) ;
	if(steeringangle > 0 )//×ó×ª
		steeringangle = steeringangle / steeringratio_l;
	else
		steeringangle = steeringangle / steeringratio_r;
#endif

	return steeringangle;
}


void calculateboundrectange(TrajPoint *traj, float width, float *l_t_x , float *l_t_y , float *l_b_x , float *l_b_y, 
							float *r_t_x , float *r_t_y , float *r_b_x , float *r_b_y)
{
	pos tmp_pos;
	tmp_pos.x = traj->x ;//m
	tmp_pos.y = traj->y ;//m

	pos lt, rt, lb, rb;
	lt.x = -width;
	lt.y = VEHICLELEN_M;
	lb.x = -width;
	lb.y = 0;
	rt.x = width;
	rt.y = VEHICLELEN_M;
	rb.x = width;
	rb.y = 0;

	*l_t_x = lt.x * sin(traj->theta) + lt.y * cos(traj->theta) + tmp_pos.x;
	*l_t_y = -lt.x * cos(traj->theta) + lt.y * sin(traj->theta) + tmp_pos.y;

	*l_b_x = lb.x * sin(traj->theta) + lb.y * cos(traj->theta) + tmp_pos.x;
	*l_b_y = -lb.x * cos(traj->theta) + lb.y * sin(traj->theta) + tmp_pos.y;

	*r_t_x = rt.x * sin(traj->theta) + rt.y * cos(traj->theta) + tmp_pos.x;
	*r_t_y = -rt.x * cos(traj->theta) + rt.y * sin(traj->theta) + tmp_pos.y;

	*r_b_x = rb.x * sin(traj->theta) + rb.y * cos(traj->theta) + tmp_pos.x;
	*r_b_y = -rb.x * cos(traj->theta) + rb.y * sin(traj->theta) + tmp_pos.y;
}





