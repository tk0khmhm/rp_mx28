#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
//#include <geometry_msgs/ChannelFloat32.h>	// for intensity, added by T.Konuma
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <unistd.h>		// for Command Line Option
#include <getopt.h>		// for Command Line Option

#include <omp.h>

#define DEG_30 M_PI/6.0
#define DEG_60 M_PI/3.0
#define THETA M_PI/6.0
#define H 0.1
#define R 0.025
#define N 1080
#define M 6

#define M_PI2 2*M_PI

using namespace Eigen;
using namespace std;

int vpn;
int hnt = 0;
double OFFSET;			// mx28のニュートラルのオフセット
double last_ang=0.0;
//double pitch = 30.0;		// [degree] コニカルの設置角度
double pitch = 32.0;		// [degree] コニカルの設置角度

tf::Transform frame;

boost::mutex ang_mutex;
boost::mutex lsr_mutex;
boost::mutex odo_mutex;
boost::mutex tf_mutex;

Matrix<double,4,N> dat;
Matrix<double,4,1> temp;
Matrix<double,M,N+3> set_d;

Matrix4d m;
Matrix4d d;
Matrix4d o = Matrix4d::Identity();

Matrix4d function_1(double phi);
Matrix4d function_2(double phi);
Matrix4d function_3(double phi);
Matrix4d function_4(double phi);

double data_set[M][N+3];
double data_intensity[M][N];	// for intensity, added by T.Konuma

sensor_msgs::LaserScan get_scan;
geometry_msgs::PoseStamped get_ang;


float tmp_1 = 0.0;
float tmp_2 = 0.0;


bool flag_ang=false;
bool flag_laser=false;
bool flag_pub=false;



////////////////SMOOTHING///////////////////
const size_t smo = 3;

double q2rpy(const tf::StampedTransform& transform);
size_t h(int n){
	if(n<0)return (size_t)0;
	if(n>(int)N)return N;
	return (size_t)n;
}
void smoothing(sensor_msgs::LaserScanConstPtr &msg){
	for(int i=0;i<msg->ranges.size();i++){
		float ave = 0.0f;
		int num = 0;
		for(int j=i-smo; j<i+smo; j++){
			if((j>=0)&&(j<N)&&(msg->ranges[h(j)]!=0.0) && !isinf(msg->ranges[h(j)])){
				ave += msg->ranges[h(j)];
				num ++;
			}
		}
		if(num!=0)
			ave /= (float)num;
		if(isinf(msg->ranges[i])/* || (num < 3)*/)
			get_scan.ranges[i] = 0.0;
		else
			get_scan.ranges[i] = ave;

	}
}
////////////////SMOOTHING///////////////////

void ang_callback(geometry_msgs::PoseStampedConstPtr msg){

		boost::mutex::scoped_lock(ang_mutex);
		get_ang = *msg;
		//cout<<get_ang<<endl;
}

void laser_callback(sensor_msgs::LaserScanConstPtr msg){
		boost::mutex::scoped_lock(lsr_mutex);
		get_scan = *msg;
		//smoothing(msg); //SMOOTHING
		//cout<<get_scan<<endl;
		//cout<<"range size: "<<get_scan.ranges.size()<<endl;
		//cout<<"intensity size: "<<get_scan.intensities.size()<<endl;
		//for(size_t i=0; i<get_scan.intensities.size(); i++){
		//	cout<<get_scan.intensities[i]<<", ";
		//}
}



Matrix4d function_1(double phi)
{
	Matrix4d rsl;
	rsl(0,0) = cos(M_PI/6.0)*cos(phi)*cos(phi)+sin(phi)*sin(phi);
	rsl(0,1) = -cos(M_PI/6.0)*cos(phi)*sin(phi)+cos(phi)*sin(phi);
	rsl(0,2) = sin(M_PI/6.0)*cos(phi);
	rsl(0,3) = 0.0;
	
	rsl(1,0) = -cos(M_PI/6.0)*cos(phi)*sin(phi)+cos(phi)*sin(phi);
	rsl(1,1) = cos(M_PI/6.0)*sin(phi)*sin(phi)+cos(phi)*cos(phi);
	rsl(1,2) = -sin(M_PI/6.0)*sin(phi);
	rsl(1,3) = 1.0;
	
	rsl(2,0) = -sin(M_PI/6.0)*cos(phi);
	rsl(2,1) = sin(M_PI/6.0)*sin(phi);
	rsl(2,2) = cos(M_PI/6.0);
	rsl(2,3) = 1.0;
	
	rsl(3,0) = 0.0;
	rsl(3,1) = 0.0;
	rsl(3,2) = 0.0;
	rsl(3,3) = 1.0;

	return rsl;
}

Matrix4d function_2(double phi)
{
	double ct = cos(THETA);
	double st = sin(THETA);
	double cp = cos(phi);
	double sp = sin(phi);
	
	Matrix4d rsl;
	
	rsl(0,0) = ct*cp*cp+sp*sp;
	rsl(1,0) = (1.0 - ct)*cp*sp;
	rsl(2,0) = st*cp;
	rsl(3,0) = 0.0;
	
	rsl(0,1) = (1.0 - ct)*cp*sp;
	rsl(1,1) = ct*sp*sp+cp*cp;
	rsl(2,1) = -st*sp;
	rsl(3,1) = 0.0;
	
	rsl(0,2) = -st*cp;
	rsl(1,2) = st*sp;
	rsl(2,2) = ct;
	rsl(3,2) = 0.0;
	
	rsl(0,3) = -R*cp;
	rsl(1,3) = R*sp;
	rsl(2,3) = H;
	rsl(3,3) = 1.0;
	
	return rsl;
}

double ct = cos(DEG_30);
double st = sin(DEG_30);
double cos_60 = cos(DEG_60);
double tan_60 = tan(DEG_60);
double tan_60_ = 1.0/tan(DEG_60);
double cp;
double sp;
double ro;
double be;
double ga;
double sin_ga;
double cos_ga;
double sin_ro;
double cos_ro;

double c = -0.076;
Matrix4d rsl;
Matrix4d function_3(double phi)
{
//	double ct = cos(DEG_30);
//	double st = sin(DEG_30);
	cp = cos(phi);
	sp = sin(phi);
//	double ro = atan(sp/tan(DEG_60));
//	double be = atan(sp*cos(DEG_60));
//	double ga = atan(cos(be)*cp/tan(DEG_60));
	ro = atan(sp*tan_60_);
	be = atan(sp*cos_60);
	ga = atan(cos(be)*cp*tan_60_);
	sin_ga = sin(ga);
	cos_ga = cos(ga);
	sin_ro = sin(ro);
	cos_ro = cos(ro);

	phi = -phi;
	
	
	//Matrix4d rsl;
	
	//rsl(0,0) = cos_ga;
	//rsl(1,0) = sin_ro*sin_ga;
	//rsl(2,0) = cos_ro*sin_ga;
	//rsl(3,0) = 0.0;
	//
	//rsl(0,1) = 0.0;
	//rsl(1,1) = cos_ro;
	//rsl(2,1) = -sin_ro;
	//rsl(3,1) = 0.0;
	//
	//rsl(0,2) = -st*cp;
	//rsl(1,2) = st*sp;
	//rsl(2,2) = ct;
	//rsl(3,2) = 0.0;
	//
	//rsl(0,3) = -R*cp;
	//rsl(1,3) = R*sp;
	//rsl(2,3) = H;
	//rsl(3,3) = 1.0;
	
	rsl(0,0) = cos(phi);
	rsl(1,0) = 0;
	rsl(2,0) = sin(phi);
	rsl(3,0) = 0.0;
	
	rsl(0,1) = 0.0;
	rsl(1,1) = 1.0;
	rsl(2,1) = 0.0;
	rsl(3,1) = 0.0;
	
	rsl(0,2) = -sin(phi);
	rsl(1,2) = 0.0;
	rsl(2,2) = cos(phi);
	rsl(3,2) = 0.0;
	
	rsl(0,3) = c*sin(phi);
	rsl(1,3) = 0.0;
	rsl(2,3) = c*cos(phi);
	rsl(3,3) = 1.0;
	
//	rsl(0,0) = cos(ga);
//	rsl(1,0) = sin(ro)*sin(ga);
//	rsl(2,0) = cos(ro)*sin(ga);
//	rsl(3,0) = 0.0;
//	
//	rsl(0,1) = 0.0;
//	rsl(1,1) = cos(ro);
//	rsl(2,1) = -sin(ro);
//	rsl(3,1) = 0.0;
//	
//	rsl(0,2) = -st*cp;
//	rsl(1,2) = st*sp;
//	rsl(2,2) = ct;
//	rsl(3,2) = 0.0;
//	
//	rsl(0,3) = -R*cp;
//	rsl(1,3) = R*sp;
//	rsl(2,3) = H;
//	rsl(3,3) = 1.0;
	
	frame.setOrigin(tf::Vector3(		rsl(0,3),
						rsl(1,3),
						rsl(2,3))
					);
	
	frame.setRotation(tf::Quaternion( 	asin(rsl(2,1)),
						atan2(-rsl(0,1),rsl(1,1)),
						atan2(-rsl(2,0),rsl(2,2)) )
					);
	
	return rsl;
}

int main(int argc, char** argv)
{


	/********* Command Line Options ************/
	char* opt_str = {(char*)"o:p:ir:c:h"};
	struct option long_opts[] = {
		{"offset",		required_argument,	0,	'o'},
		{"pitch",		required_argument,	0,	'p'},
		{"intensity",	required_argument,	0,	'i'},
		{"intence_remove",	required_argument,	0,	'r'},
		{"clearance",	required_argument,	0,	'c'},
		{"help",		no_argument,		0,	'h'},
		{0,				0,					0,    0}
	};
	int opt_idx = 0;
	char *buff;

	//double offset = 180.0;
	double offset = 181.0;
	bool intensity_flag = false;
	double intensity_threshold = 0.0;
	double clearance = 0.0;

	int c;
	while( (c = getopt_long(argc, argv, opt_str, long_opts, &opt_idx)) != -1 ){
		switch (c){
			case 'o':
				offset = (double)atof(optarg);
				cout<<"offset:"<<offset<<endl;
				break;
			case 'p':
				pitch = (double)atof(optarg);
				cout<<"pitch:"<<pitch<<endl;
				break;
			case 'i':
				intensity_flag = true;
				cout<<"intensity_flag:"<<intensity_flag<<endl;
				break;
			case 'r':
				intensity_threshold = (double)atof(optarg);
				cout<<"intensity_threshold:"<<intensity_threshold<<endl;
				break;
			case 'c':
				clearance = (double)atof(optarg);
				cout<<"clearance:"<<clearance<<endl;
				break;
			case 'h':
				puts("");
				printf("Usage: rosrun rp_mx28 conical_high_speed_eth [options]\n");
				puts("");
				printf("Option:     Option list of the available\n");
				printf("	--offset			-o:\n");
				printf("	--pitch				-p:\n");
				printf("	--intensity			-i:\n");
				printf("	--intence_remove	-r:\n");
				printf("	--clearance			-c:\n");
				printf("    --help          -h:\n");
				puts("");
				exit(0);
			default:
				fprintf(stderr, "Error: An unknown option is appointed.\n");
				exit(1);
		}
	}


	/*******************************************/


	//OFFSET = 275.0;	//speed=0.25
	//OFFSET = 90.0;
	//OFFSET = 180.0;
	OFFSET = offset;
	//cin>>OFFSET ;	//speed=0.25


	ros::init(argc, argv, "conical_check_eth");
	ros::NodeHandle n;
	ros::Rate roop(40);
	ros::Publisher pub_2 = n.advertise<sensor_msgs::PointCloud>("eth_top/point_cloud_2", 1);
	ros::Publisher pub_3 = n.advertise<sensor_msgs::PointCloud>("eth_top/point_cloud_3", 1);
	ros::Subscriber ang_sub = n.subscribe("conical/ang", 10, ang_callback);
	//ros::Subscriber laser_sub = n.subscribe("/most_intense", 10, laser_callback);
	//ros::Subscriber laser_sub = n.subscribe("/eth_top/scan", 10, laser_callback);
	ros::Subscriber laser_sub = n.subscribe("eth_top/scan", 10, laser_callback);
	tf::TransformListener listener;
  	tf::TransformBroadcaster br;
	sensor_msgs::PointCloud pc_2;
	sensor_msgs::PointCloud pc_3;
	pc_2.header.frame_id = "/3dlaser";
	pc_3.header.frame_id = "/3dlaser";
	pc_3.channels.resize(1);	// for intensity, added by T.Konuma
	pc_3.channels[0].name = "intensity";	// for intensity, added by T.Konuma
	pc_2.points.resize(4);	
	get_scan.ranges.resize(N);	
	get_scan.intensities.resize(N);

//////////////////////////////////////////////////////////////////////rotation θdeg
	pitch=M_PI*pitch/180.0;		// defineしたpitchをdegreeからradianへと変換
	Matrix4d y;					// 設置の回転行列
	y(0,0) = cos(pitch);
	y(0,1) = 0;
	y(0,2) = sin(pitch);
	
	y(1,0) = 0;
	y(1,1) = 1;
	y(1,2) = 0;
	
	y(2,0) = -sin(pitch);
	y(2,1) = 0;
	y(2,2) = cos(pitch);

	y(1,3) = 0;
	y(2,3) = 0;
	y(3,3) = 0;

	y(3,0) = 0;
	y(3,1) = 0;
	y(3,2) = 0;
	y(3,3) = 0;

	//cout<<"y:"<<y<<endl;

////////////////////////////////////////////////////////////////////////
	
	int pc_count = 0;
	int ang_count = 0;
	
	int i, j, ii;
	double ang;
	double increment;
	int chose_num;
	double theta;
	double range;
	double k;
	int counter;

	//#ifdef _OPENMP
	//#pragma omp parallel while
	//#endif
	while (ros::ok()){
//////////////////////////////////////////////////////////////////////make_data_set
		//cout<<"laser data buffering"<<endl;
		{
			boost::mutex::scoped_lock(lsr_mutex);
			for(i=0;i<N;i++){
				data_set[0][i] = get_scan.ranges[i];
				//cout<<"i:"<<i;
				//cout<<",  ranges:"<<get_scan.ranges[i];
				//cout<<", intencity size:"<<get_scan.intensities.size();
				//cout<<",  intencity:"<<get_scan.intensities[i];
				data_intensity[0][i] = get_scan.intensities[i];	// for intensity, added by T.Konuma
			}
			data_set[0][N] = get_scan.header.stamp.toNSec();
			pc_count++;
		}

		
		{
			boost::mutex::scoped_lock(ang_mutex);
			data_set[0][N+1] = get_ang.pose.position.x;
			data_set[0][N+2] = get_ang.header.stamp.toNSec();
			ang_count++;
		}

//////////////////////////////////////////////////////////////////////pub_count
		if((pc_count > 10) && (ang_count > 10)){
			flag_pub = true;
		//	cout<<"a"<<endl;
		}else{
		//	cout<<"----------------------------------wait"<<endl;
		}
//////////////////////////////////////////////////////////////////////make_ang
		ang = data_set[M/2-1][N+1] + OFFSET*M_PI/180.0;	// なぜ[M/2-1]?
		ang *= -1.0;				// mx28のZ軸が逆向きだから変換
		//if(ang<0.0)ang+=2*M_PI;		// angは0~3.14まで
		if(ang<0.0)ang+=M_PI2;		// angは0~3.14まで
		
		last_ang = data_set[M/2][N+1] + OFFSET*M_PI/180.0;
		last_ang *= -1.0;
		//if(last_ang<0.0)last_ang+=2*M_PI;
		if(last_ang<0.0)last_ang+=M_PI2;


//////////////////////////////////////////////////////////////////////chose_ranges
		chose_num=0;
		increment=1000000000.0;	//backup	// urgのtime stampとmx28のtime stampの近いやつを探す
		//double increment=500000000.0;
		for(i=0;i<M;i++){
			if(increment > fabs(data_set[M/2-1][N+2]-data_set[i][N])){	// N:laser's time, N+2:ang's time
				increment = fabs(data_set[M/2-1][N+2]-data_set[i][N]);
				chose_num=i;
			}
		}
//////////////////////////////////////////////////////////////////////make_range
		// polar to xyzw

		//#ifdef _OPENMP
		//#pragma omp parallel for
		//#endif
		for(i=0;i<N;i++){
			range = data_set[chose_num][i];
				
			if(isinf(range)){
				dat(0,i)=0.0;	// dat : Matrix<double,4,N>
				dat(1,i)=0.0;	
			}else if(isnan(range)){
				dat(0,i) = 0.0;
				dat(1,i) = 0.0;
			}else if(range > 0.1){
				//theta = 1.5*M_PI*(double)i/(double)(N-1)-M_PI*3.0/4.0;
				theta = 1.5*M_PI*(double)i/(double)(N-1)-M_PI*0.75;
				dat(0,i)=range*cos(theta);
				dat(1,i)=range*sin(theta);
			}else{
				dat(0,i)=0.0;
				dat(1,i)=0.0;					
			}
			vpn=N;	// not used ...?
			dat(2,i)=0.0;
			dat(3,i)=1.0;
		}
//////////////////////////////////////////////////////////////////////make_d,dat			
		counter = 0;
		//cout<<"ang-last_ang:"<<ang-last_ang<<endl;
		for(ii=0;ii<N;ii++){
			k=0.0;	// laser1本の角度

			//if(fabs(ang-last_ang)>0.6) k=last_ang+2*M_PI+(-2*M_PI+(ang-last_ang))/1440.0*(double)(i+180);
			if(fabs(ang-last_ang)>0.6){
				k=last_ang+M_PI2+(-M_PI2+(ang-last_ang))/1440.0*(double)(ii+180);
			}else{
				k=last_ang+(ang-last_ang)/1440.0*(double)(ii+180);
			}


			//cout<<"k:"<<k<<endl;
			d = function_3(k);	// d:Matrix4d, function_3:回転行列?
			for(j=0;j<3;j++){
				temp(j,0)=dat(j,ii);	//temp:Matrix<double,4,1>
			}
			temp(3.0)=1.0;
			//cout<<"temp:"<<endl;
			temp=d*temp;
			//cout<<"d:"<<d<<endl;
			//cout<<"d*temp:"<<temp<<endl;
			//temp=y*temp;	// conicalの設置角度による座標変換
			//cout<<"y:"<<y<<endl;
			//cout<<"y*temp:"<<temp<<endl;
//////////////////////////////////////////////////////////////////////make_pc2,3
			for(i=0;i<3;i++){
				pc_2.points[i].x = d(0,i)+d(0,3);
				pc_2.points[i].y = d(1,i)+d(1,3);
				pc_2.points[i].z = d(2,i)+d(2,3);
			}
			pc_2.points[3].x = d(0,3);
			pc_2.points[3].y = d(1,3);
			pc_2.points[3].z = d(2,3);		
			
			//if((sqrt(pow(temp(0,0),2)+pow(temp(1,0),2)+pow(temp(2,0),2) ) > 1.0)/* && (temp(2.0) > -0.2)*/){			//curv を使わないときはOK
			//if((sqrt(pow(temp(0,0),2)+pow(temp(1,0),2)+pow(temp(2,0),2) ) > 0.09)/* && (temp(2.0) > -0.2)*/){			//curv を使わないときはOK
			if(data_set[chose_num][ii]>clearance && get_scan.intensities[ii]>intensity_threshold){
				geometry_msgs::Point32 p;
				p.x = temp(0,0);
				p.y = temp(1,0);
				p.z = temp(2,0);
				pc_3.points.push_back(p);	
				pc_3.channels[0].values.push_back(get_scan.intensities[ii]);	// for intensity, added by T.Konuma
				counter++;
			}else{
				geometry_msgs::Point32 pp;
				pp.x = 0.0;
				pp.y = 0.0;
				pp.z = 0.0;
				pc_3.points.push_back(pp);
				pc_3.channels[0].values.push_back(0);	// for intensity, added by T.Konuma
			}
		}
//////////////////////////////////////////////////////////////////////pub_pc2,3
		if(increment > 20000000.0 || flag_pub==false);
		else{
        	pc_2.header.stamp = ros::Time::now();
        	pc_3.header.stamp = ros::Time::now();
			pub_2.publish(pc_2);
			if(pc_3.points.size() != 0)
			pub_3.publish(pc_3);
			//cout<<counter<<" ( "<<(int)((double)counter/(double)N*100.0)<<" % )"<<endl;	
		}
		pc_3.points.clear();
		pc_3.channels[0].values.clear();	// for intensity, added by T.Konuma
//////////////////////////////////////////////////////////////////////rotation
		//cout<<"laser data backup"<<endl;
		for(i=M-1;i>=1;i--){
			for(j=0;j<N+3;j++){
				data_set[i][j]=data_set[i-1][j];
				if(j<N){
					//cout<<"i:"<<i;
					data_intensity[i][j] = data_intensity[i-1][j];	// for intensity, added by T.Konuma
					//cout<<"    end"<<endl;
				}
			}
		}
//////////////////////////////////////////////////////////////////////fin
		ros::spinOnce();
		roop.sleep();
	}
}
