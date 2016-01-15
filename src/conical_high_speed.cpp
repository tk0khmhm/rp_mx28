#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <time.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <omp.h>

#define DEG_30 M_PI/6.0
#define DEG_60 M_PI/3.0
#define THETA M_PI/6.0
#define H 0.1
#define R 0.025
#define N 1080
#define M 6

using namespace Eigen;
using namespace std;

int vpn;
int hnt = 0;
double OFFSET;
double last_ang=0.0;
double pitch = 0.0;

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

Matrix4d function_3(double phi)
{
	double ct = cos(DEG_30);
	double st = sin(DEG_30);
	double cp = cos(phi);
	double sp = sin(phi);
	double ro = atan(sp/tan(DEG_60));
	double be = atan(sp*cos(DEG_60));
	double ga = atan(cos(be)*cp/tan(DEG_60));
	
	
	Matrix4d rsl;
	
	rsl(0,0) = cos(ga);
	rsl(1,0) = sin(ro)*sin(ga);
	rsl(2,0) = cos(ro)*sin(ga);
	rsl(3,0) = 0.0;
	
	rsl(0,1) = 0.0;
	rsl(1,1) = cos(ro);
	rsl(2,1) = -sin(ro);
	rsl(3,1) = 0.0;
	
	rsl(0,2) = -st*cp;
	rsl(1,2) = st*sp;
	rsl(2,2) = ct;
	rsl(3,2) = 0.0;
	
	rsl(0,3) = -R*cp;
	rsl(1,3) = R*sp;
	rsl(2,3) = H;
	rsl(3,3) = 1.0;
	
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



	//OFFSET = 275.0;	//speed=0.25
	//OFFSET = 135.0;
	cin>>OFFSET ;	//speed=0.25


	ros::init(argc, argv, "conical_check");
	ros::NodeHandle n;
	ros::Rate roop(40);
	ros::Publisher pub_2 = n.advertise<sensor_msgs::PointCloud>("point_cloud_2", 1);
	ros::Publisher pub_3 = n.advertise<sensor_msgs::PointCloud>("point_cloud_3", 1);
	ros::Subscriber ang_sub = n.subscribe("/ang", 10, ang_callback);
	ros::Subscriber laser_sub = n.subscribe("/scan", 10, laser_callback);
	tf::TransformListener listener;
  	tf::TransformBroadcaster br;
	sensor_msgs::PointCloud pc_2;
	sensor_msgs::PointCloud pc_3;
	pc_2.header.frame_id = "/3dlaser";
	pc_3.header.frame_id = "/3dlaser";
	pc_2.points.resize(4);	
	get_scan.ranges.resize(N);	

//////////////////////////////////////////////////////////////////////rotation θdeg
	pitch=M_PI*pitch/180.0;
	Matrix4d y;
	y(0,0) = cos(pitch);
	y(0,1) = 0;
	y(0,2) = sin(pitch);
	
	y(1,0) = 0;
	y(1,1) = 1;
	y(1,2) = 0;
	
	y(2,0) = -sin(pitch);
	y(2,1) = 0;
	y(2,2) = cos(pitch);

////////////////////////////////////////////////////////////////////////
	
	int pc_count = 0;
	int ang_count = 0;
	
	while (ros::ok()){
//////////////////////////////////////////////////////////////////////make_data_set
		{
			boost::mutex::scoped_lock(lsr_mutex);
			for(int i=0;i<N;i++){
				data_set[0][i] = get_scan.ranges[i];
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
		double ang = data_set[M/2-1][N+1] + OFFSET*M_PI/180.0;
		ang *= -1.0;
		if(ang<0.0)ang+=2*M_PI;
		
		last_ang = data_set[M/2][N+1] + OFFSET*M_PI/180.0;
		last_ang *= -1.0;
		if(last_ang<0.0)last_ang+=2*M_PI;


//////////////////////////////////////////////////////////////////////chose_ranges
		int chose_num=0;
		double increment=1000000000.0;	//backup
		//double increment=500000000.0;
		for(int i=0;i<M;i++){
			if(increment > fabs(data_set[M/2-1][N+2]-data_set[i][N])){
				increment = fabs(data_set[M/2-1][N+2]-data_set[i][N]);
				chose_num=i;
			}
		}
//////////////////////////////////////////////////////////////////////make_range
		double theta;
		double range;

		//#ifdef _OPENMP
		//#pragma omp parallel for
		//#endif
		for(int i=0;i<N;i++){
			range = data_set[chose_num][i];
				
			if(isinf(range)){
				dat(0,i)=0.0;
				dat(1,i)=0.0;	
			}else if(isnan(range)){
				dat(0,i) = 0.0;
				dat(1,i) = 0.0;
			}else if(range > 0.1){
				theta = 1.5*M_PI*(double)i/(double)(N-1)-M_PI*3.0/4.0;
				dat(0,i)=range*cos(theta);
				dat(1,i)=range*sin(theta);
			}else{
				dat(0,i)=0.0;
				dat(1,i)=0.0;					
			}
			vpn=N;
			dat(2,i)=0.0;
			dat(3,i)=1.0;
		}
//////////////////////////////////////////////////////////////////////make_d,dat			
		int counter = 0;
		for(int i=0;i<N;i++){
			double k=0.0;

			if(fabs(ang-last_ang)>0.6) k=last_ang+2*M_PI+(-2*M_PI+(ang-last_ang))/1440.0*(double)(i+180);
			else k=last_ang+(ang-last_ang)/1440.0*(double)(i+180);


			//cout<<k<<endl;
			d = function_3(k);
			for(int j=0;j<3;j++){
				temp(j,0)=dat(j,i);
			}
			temp=d*temp;
			//temp=y*temp;
			//cout<<temp<<endl;
//////////////////////////////////////////////////////////////////////make_pc2,3
			for(int i=0;i<3;i++){
				pc_2.points[i].x = d(0,i)+d(0,3);
				pc_2.points[i].y = d(1,i)+d(1,3);
				pc_2.points[i].z = d(2,i)+d(2,3);
			}
			pc_2.points[3].x = d(0,3);
			pc_2.points[3].y = d(1,3);
			pc_2.points[3].z = d(2,3);		
			
			//if((sqrt(pow(temp(0,0),2)+pow(temp(1,0),2)+pow(temp(2,0),2) ) > 1.0)/* && (temp(2.0) > -0.2)*/){			//curv を使わないときはOK
			if((sqrt(pow(temp(0,0),2)+pow(temp(1,0),2)+pow(temp(2,0),2) ) > 0.09)/* && (temp(2.0) > -0.2)*/){			//curv を使わないときはOK
				geometry_msgs::Point32 p;
				p.x = temp(0,0);
				p.y = temp(1,0);
				p.z = temp(2,0);
				pc_3.points.push_back(p);	
				counter++;
			}else{
				geometry_msgs::Point32 pp;
				pp.x = 0.0;
				pp.y = 0.0;
				pp.z = 0.0;
				pc_3.points.push_back(pp);
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
//////////////////////////////////////////////////////////////////////rotation
		for(int i=M-1;i>=1;i--){
			for(int j=0;j<N+3;j++){
				data_set[i][j]=data_set[i-1][j];
			}
		}
//////////////////////////////////////////////////////////////////////fin
		ros::spinOnce();
		roop.sleep();
	}
}
