#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <boost/thread.hpp>

#include <unistd.h>		// for Command Line Option
#include <getopt.h>		// for Command Line Option

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

#include <wiringPi.h>
#include <wiringSerial.h>


#define	PORT		"/dev/ttyAMA0"	// port of raspberry pi
#define MX28_ID		0				// id of mx28 (0~253)
#define BAUDRATE	57600			// baudrate
#define T_PIN		27				// 20lx(laser)からのタイミングパルスを受け取るピン
#define MAX_ANG		M_PI*210.0/180.0
#define MIN_ANG		M_PI*150.0/180.0

#define EEPROM_ID							0x03	// 1byte
#define EEPROM_BAUDRATE						0x04	// 1byte
#define EEPROM_CW_ANGLE_LIMIT				0x06	// 2byte
#define EEPROM_CCW_ANGLE_LIMIT				0x08	// 2byte
#define EEPROM_HIGHEST_LIMIT_TEMPERATURE	0x0B	// 1byte
#define EEPROM_LOWEST_LIMIT_VOLTAGE			0x0C	// 1byte
#define EEPROM_HIGHEST_LIMIT_VOLTAGE		0x0D	// 1byte

#define RAM_GOAL_POSITION					0x1E	// 2byte
#define RAM_MOVING_SPEED					0x20	// 2byte
#define RAM_PRESENT_POSITION				0x24	// 1byte
#define RAM_PRESENT_SPEED					0x26	// 1byte
#define RAM_PRESENT_LOAD					0x28	// 2byte
#define RAM_PRESENT_VOLTAGE					0x2A	// 1byte
#define RAM_PRESENT_TEMPERATURE				0x2B	// 1byte
#define RAM_GOAL_ACCELERATION				0x49	// 1byte



using namespace std;

char id;
int fd;
bool timing_flag=0;


void MX28_write(char ID, int start, int bytes, char* data, int fd)
{
	char TxBuf[16];
	char sum = 0;
	char Status[8];

	TxBuf[0] = 0xff;
	TxBuf[1] = 0xff;
	
	TxBuf[2] = ID;
	sum += TxBuf[2];

	TxBuf[3] = 3+bytes;
	sum += TxBuf[3];

	TxBuf[4] = 0x03;
	sum += TxBuf[4];

	TxBuf[5] = start;
	sum += TxBuf[5];

	for(char i=0; i<bytes; i++){
		TxBuf[6+i] = data[i];
		sum += TxBuf[6+i];
	}

	TxBuf[6+bytes] = 0xff - sum;

	for(int i=0; i<(7+bytes); i++){
		//printf("%d ", TxBuf[i]);
		serialPutchar(fd, TxBuf[i]);
	}
	puts("");

	//for(int i=0; i<15; i++){
	//for(int i=0; i<6; i++){
	int status_length = 6;
	for(int i=0; i<status_length; i++){
		Status[i] = serialGetchar(fd);
		//printf("%d ", Status[i]);
		if(i==3){
			status_length = Status[3] + 4;
		}
	}
	puts("");
}

void MX28_read(char id, int head_address, int fd)
{
	char TxBuf[16];
	char sum = 0;
	char RxBuf[16];

	TxBuf[0] = 0xff;
	TxBuf[1] = 0xff;
	
	//TxBuf[2] = 0x01;
	TxBuf[2] = id;
	sum += TxBuf[2];

	TxBuf[3] = 0x04;
	sum += TxBuf[3];

	TxBuf[4] = 0x02;
	sum += TxBuf[4];

	TxBuf[5] = head_address;
	sum += TxBuf[5];

	TxBuf[6] = 0x02;
	sum += TxBuf[6];

	TxBuf[7] = 0xff - sum;

	for(int i=0; i<8; i++){
		serialPutchar(fd, TxBuf[i]);
	}
	//for(int i=0; i<16; i++){
	for(int i=0; i<8; i++){
		RxBuf[i] = serialGetchar(fd);
		printf("%d ", RxBuf[i]);
	}
}	

void SetID(char change_id, int fd)
{
	//MX28_write(0xfe, 0x03, 1, &change_id, fd);	// 0xfe:全てのmx28
	MX28_write(0xfe, EEPROM_ID, 1, &change_id, fd);	// 0xfe:全てのmx28
}

void SetBAUD(char id, int change_baud, int fd)
{
	char change_baud_;
	switch (change_baud) {
		case 9600 :
			change_baud_ = 207;
			break;
		case 19200 :
			change_baud_ = 103;
			break;
		case 57600 :
			change_baud_ = 34;
			break;
		case 115200 :
			change_baud_ = 16;
			break;
	}
	//MX28_write(id, 0x04, 1, &change_baud_, fd);
	MX28_write(id, EEPROM_BAUDRATE, 1, &change_baud_, fd);
}

void SetJointMode(char id, int fd)
{
	short data = 0;
	char byte[2];
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	MX28_write(id, EEPROM_CW_ANGLE_LIMIT, 2, byte, fd);

	data = 4095;
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	MX28_write(id, EEPROM_CCW_ANGLE_LIMIT, 2, byte, fd);
}

void SetWheelMode(char id, int fd)
{
	short data = 0;
	char byte[2];
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	MX28_write(id, EEPROM_CW_ANGLE_LIMIT, 2, byte, fd);
	MX28_write(id, EEPROM_CCW_ANGLE_LIMIT, 2, byte, fd);
}

void SetCWAngleLimit(char id, double cw_angle_limit, int fd)
{
	short data;
	char byte[2];
	data = (short)( cw_angle_limit * 4096.0 / 360.0 );
	cout<<"cw_limit:"<<data<<endl;
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	MX28_write(id, EEPROM_CW_ANGLE_LIMIT, 2, byte, fd);
}

void SetCCWAngleLimit(char id, double ccw_angle_limit, int fd)
{
	short data;
	char byte[2];
	data = (short)( ccw_angle_limit * 4096.0 / 360.0 );
	cout<<"ccw_limit:"<<data<<endl;
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	MX28_write(id, EEPROM_CCW_ANGLE_LIMIT, 2, byte, fd);
}

void SetGoalPosition(char id, double goal_position, int fd)
{
	short data;
	char byte[2];
	data = (short)( goal_position * 4096.0 / 360.0 );
	cout<<"goal_position:"<<data<<endl;
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	MX28_write(id, RAM_GOAL_POSITION, 2, byte, fd);
}

//void MX28SetMovingSpeed(float rpm, int fd)
void MX28SetMovingSpeed(char id, float rpm, int fd)
{
	short data;
	char byte[2];
	data = (int)(fabs(rpm)/0.1144);
	byte[0] = 0xff & data;
	byte[1] = data >> 8;
	if(rpm<0){
		byte[1] |= 0x04;
	}
	MX28_write(id, RAM_MOVING_SPEED, 2, byte, fd);
	printf("setted moving speed : %f\n", rpm);
	//MX28_write(id, 0x20, 2, byte, fd);
	//MX28_write(0x01, 0x20, 2, byte, fd);
}


float ReadPresentPosition(char id, int fd)
{
	char TxBuf[16];
	char sum = 0;
	char RxBuf[16];

	TxBuf[0] = 0xff;
	TxBuf[1] = 0xff;
	
	//TxBuf[2] = 0x01;
	TxBuf[2] = id;
	sum += TxBuf[2];

	TxBuf[3] = 0x04;
	sum += TxBuf[3];

	TxBuf[4] = 0x02;
	sum += TxBuf[4];

	TxBuf[5] = 0x24;
	sum += TxBuf[5];

	TxBuf[6] = 0x02;
	sum += TxBuf[6];

	TxBuf[7] = 0xff - sum;

	for(int i=0; i<8; i++){
		serialPutchar(fd, TxBuf[i]);
	}
	//for(int i=0; i<16; i++){
	for(int i=0; i<8; i++){
		RxBuf[i] = serialGetchar(fd);
		//printf("%d ", RxBuf[i]);
	}
	//puts("");

	//float radian = (RxBuf[13] + RxBuf[14]*256.0) * 2.0 * M_PI / 4096.0 ;
	//float radian = (RxBuf[5] + RxBuf[6]*256.0) * 2.0 * M_PI / 4096.0 ;
	float radian = (RxBuf[5] + RxBuf[6]*256.0) * 0.00153398 ;
	//if(radian > M_PI){
	//	radian -= 2.0*M_PI;
	//}
	return radian;
	//float degree = (RxBuf[13] + RxBuf[14]*256.0) * 360.0 / 4096.0;
	//return degree;
}	

void SetGoalAcceleration(char id, double goal_acceleration, int fd)
{
	short data;
	char byte[2];
	data = (short)( goal_acceleration / 8.583 );
	cout<<"goal_acceleration:"<<data<<endl;
	byte[0] = 0xff & data;
	MX28_write(id, RAM_GOAL_ACCELERATION, 1, byte, fd);
}



boost::mutex cmd_mutex;

void cmd_callback(const std_msgs::Float32ConstPtr& msg)
{
	boost::mutex::scoped_lock(cmd_mutex);
	printf("Set moving speed %f !!!!\n", msg->data);
	MX28SetMovingSpeed(id, msg->data, fd);
}


//7/27//ros::Publisher position_pub;
//7/27//geometry_msgs::PoseStamped degree;
void interrupt(void)
{
	timing_flag = 1;
}
//7/27//void interrupt(void)
//7/27//{
//7/27//	//puts("interrupt!");
//7/27//	//ReadPresentPosition();
//7/27//
//7/27//	degree.header.stamp = ros::Time::now();
//7/27//	degree.pose.position.x = ReadPresentPosition(id, fd);
//7/27//
//7/27//	//cout<<degree.pose.position.x<<endl;
//7/27//	
//7/27//	position_pub.publish(degree);
//7/27//}

//void signal_handle(int)
//{
//	MX28SetMovingSpeed(0, fd);
//	exit(0);
//}


int main(int argc, char** argv)
{
	/********* Command Line Options ************/
	char* opt_str = {(char*)"p:i:y:b:z:jwl:m:r:d:t:a:o:e:f:h"};
	struct option long_opts[] = {
		{"port",		required_argument,	0,	'p'},
		{"id",			required_argument,	0,	'i'},
		{"change-id",	required_argument,	0,	'y'},
		{"baudrate",	required_argument,	0,	'b'},
		{"change-baud",	required_argument,	0,	'z'},
		{"joint-mode",	required_argument,	0,	'j'},
		{"wheel-mode",	required_argument,	0,	'w'},
		{"cwlimit",		required_argument,	0,	'l'},
		{"ccwlimit",	required_argument,	0,	'm'},
		{"rpm",			required_argument,	0,	'r'},
		{"degree",		required_argument,	0,	'd'},
		{"timing",		required_argument,	0,	't'},
		{"accel",		required_argument,	0,	'a'},
		{"read-memory",	required_argument,	0,	'o'},
		{"max_ang",		required_argument,	0,	'e'},
		{"min_ang",		required_argument,	0,	'f'},
		{"help",		no_argument,		0,	'h'},
		{0,				0,					0,    0}
	};
	int opt_idx = 0;
	char *buff;

	char* port = {(char*)PORT};
	id = MX28_ID;
	char change_id = 255;
	int baudrate = BAUDRATE;
	int change_baud = 0;
	bool jm_f = 0;
	bool wm_f = 0;
	double cw_limit = 0;
	bool cw_f = 0;
	double ccw_limit = 0;
	bool ccw_f = 0;
	double rpm = 0;
	double goal_position = 0;
	bool gp_f = 0;
	int t_pin = T_PIN;
	bool ga_f = 0;
	double accel = 0;
	int head_address = 0;
	bool rm_f = 0;
	double max_ang = MAX_ANG;
	double min_ang = MIN_ANG;

	int c;
	while( (c = getopt_long(argc, argv, opt_str, long_opts, &opt_idx)) != -1 ){
		switch (c){
			case 'p':
				port = optarg;
				cout<<"port:"<<port<<endl;
				break;
			case 'i':
				id = (char)strtol(optarg, &buff, 10);
				cout<<"id:"<<id<<endl;
				if(*buff != '\0'){
					fprintf(stderr, "Error: The value of the --id must be in the range [0,253].\n");
					exit(1);
				}
				break;
			case 'y':
				change_id = (char)strtol(optarg, &buff, 10);
				cout<<"changeid:"<<change_id<<endl;
				if(*buff != '\0'){
					fprintf(stderr, "Error: The value of the --id must be in the range [0,253].\n");
					exit(1);
				}
				break;
			case 'b':
				baudrate = (int)strtol(optarg, &buff, 10);
				cout<<"baudrate:"<<baudrate<<endl;
				if(*buff != '\0'){
					fprintf(stderr, "Error: The value of the --baudrate must be below number.\n");
					fprintf(stdout, "		9600, 19200, 57600, 115200\n");
					exit(1);
				}
				break;
			case 'z':
				change_baud = (int)strtol(optarg, &buff, 10);
				cout<<"change baudrate:"<<change_baud<<endl;
				if(*buff != '\0'){
					fprintf(stderr, "Error: The value of the --baudrate must be below number.\n");
					fprintf(stdout, "		9600, 19200, 57600, 115200\n");
					exit(1);
				}
				break;
			case 'j':
				jm_f = 1;
				cout<<"joint mode is selected!!"<<endl;
				break;
			case 'w':
				wm_f = 1;
				cout<<"wheel mode is selected!!"<<endl;
				break;
			case 'l':
				cw_limit = (double)atof(optarg);
				cw_f = 1;
				cout<<"cw_limit:"<<cw_limit<<endl;
				break;
			case'm':
				ccw_limit = (double)atof(optarg);
				ccw_f = 1;
				cout<<"ccw_limit:"<<ccw_limit<<endl;
				break;
			case 'r':
				rpm = (double)atof(optarg);
				cout<<"rpm:"<<rpm<<endl;
				break;
			case 'd':
				goal_position = (double)atof(optarg);
				gp_f = 1;
				cout<<"goal_position:"<<goal_position<<endl;
				break;
			case 't':
				t_pin = (int)strtol(optarg, &buff, 10);
				if(*buff != '\0'){
					fprintf(stderr, "Error: The value of the --timing must be available port.\n");
					exit(1);
				}
				break;
			case 'a':
				accel = (double)atof(optarg);
				ga_f = 1;
				cout<<"goal_acceleration:"<<accel<<endl;
				break;
			case 'o':
				head_address = (int)strtol(optarg, &buff, 10);
				rm_f = 1;
				cout<<"read-memory:"<<head_address<<endl;
				break;
			case 'e':
				max_ang = (double)atof(optarg)*3.141592/180.0;
				if(max_ang>3.141592*359/180){
					max_ang=3.141592*359/180;
				}
				cout<<"max_ang:"<<max_ang<<endl;
				break;
			case 'f':
				min_ang = (double)atof(optarg)*3.141592/180.0;
				if(min_ang<3.141592*1/180){
					min_ang=3.141592*1/180;
				}
				cout<<"min_ang:"<<min_ang<<endl;
				break;
			case 'h':
				puts("");
				printf("Usage: rosrun rp_mx28 mx28_kaizoutyuu [options]\n");
				puts("");
				printf("Option:     Option list of the available\n");
				printf("    --port      -p: [string] Set port. (-p /dev/ttyUSB0)\n");
				printf("                             Default is \"/dev/ttyAMA0\"\n");
				printf("    --id            -i: [int]    Set id. (-i 0)\n");
				printf("    --changeid      -y: [int]    Change id. (-y 23)\n");
				printf("    --baudrate      -b: [int]    Set baudrate. (-b 9600/19200,57600,115200\n");
				printf("    --changebaud    -z: [int]	 Change baudrate. (-z 115200)\n");
				printf("    --joint-mode    -j: [none]\n");
				printf("    --wheel-mode    -w: [none]\n");
				printf("    --cwlimit       -l: [double] \n");
				printf("    --ccwlimit      -m: [double] \n");
				printf("    --rpm           -r:\n");
				printf("    --degree        -d:\n");
				printf("    --timing        -t:\n");
				printf("    --accel         -a:\n");
				printf("    --read-memory   -o:\n");
				printf("    --help          -h:\n");
				puts("");
				exit(0);
			default:
				fprintf(stderr, "Error: An unknown option is appointed.\n");
				exit(1);
		}
	}


	/*******************************************/




	ros::init(argc, argv, "mx28_test");
	ros::NodeHandle n;
	ros::Subscriber speed_sub = n.subscribe("conical/cmd", 1, cmd_callback);
	ros::Publisher position_pub = n.advertise<geometry_msgs::PoseStamped>("conical/ang", 1);
	//position_pub = n.advertise<geometry_msgs::PoseStamped>("ang", 1);
	geometry_msgs::PoseStamped degree;

	//cout<<degree.pose.position.x<<endl;
	

	cout<<"port:"<<port<<endl;
	cout<<"baudrate:"<<baudrate<<endl;

	fd = serialOpen(port, baudrate);
	if(fd<0){
		fprintf(stderr, "Error: Can't open serial port\n");
		return 1;
	}

	if(change_id != 255){
			cout<<"a"<<endl;
		SetID(change_id, fd);
			cout<<"b"<<endl;
		return 0;
	}

	if(change_baud){
		cout<<"Changing baudrate"<<endl;
		SetBAUD(id, change_baud, fd);
		return 0;
	}

	if(jm_f){
		cout<<"Setting joint mode"<<endl;
		SetJointMode(id, fd);
		cout<<"Finished!!"<<endl;
		return 0;
	}
	if(wm_f){
		cout<<"Setting wheel mode"<<endl;
		SetWheelMode(id, fd);
		cout<<"Finished!!"<<endl;
		return 0;
	}
	if(cw_f){
		cout<<"Changing CW_LIMIT"<<endl;
		SetCWAngleLimit(id, cw_limit, fd);
		return 0;
	}
	if(ccw_f){
		cout<<"Changing CCW_LIMIT"<<endl;
		SetCCWAngleLimit(id, ccw_limit, fd);
		return 0;
	}
	if(gp_f){
		cout<<"goal_position"<<endl;
		SetGoalPosition(id, goal_position, fd);
		return 0;
	}
	if(ga_f){
		cout<<"goal_acceleration"<<endl;
		SetGoalAcceleration(id, accel, fd);
		return 0;
	}
	if(rm_f){
		cout<<"read_memory"<<endl;
		MX28_read(id, head_address, fd);
		return 0;
	}

	cout<<"rpm:"<<rpm<<endl;
	MX28SetMovingSpeed(id, rpm, fd);


	if(wiringPiSetupSys() == -1){
		puts("wiringPi has not been initialised!");
	}

	//wiringPiISR( 26, INT_EDGE_SETUP, interrupt);
	wiringPiISR( 26, INT_EDGE_FALLING, interrupt);


	//geometry_msgs::PoseStamped degree;
	
	while(ros::ok()){
		if(timing_flag){
			degree.header.stamp = ros::Time::now();
			degree.pose.position.x = ReadPresentPosition(id, fd);
			position_pub.publish(degree);
			timing_flag = 0;
		}

		//while(serialDataAvail(fd)){
		//	printf("%d ", serialGetchar(fd) );
		//	fflush(stdout);
		//}

		//serialPuts(fd, "hello world\n");

		//float degree = ReadPresentPosition();
		//degree.header.stamp = ros::Time::now();
		//degree.pose.position.x = ReadPresentPosition();
		//printf("%f\n",degree);

		//position_pub.publish(degree);

		//delay(100);
		//rpm = -rpm;
		//MX28SetMovingSpeed(id, rpm, fd);
		//interrupt();
		ros::spinOnce();
	}

	MX28SetMovingSpeed(id, 0, fd);

	return 0;
}
