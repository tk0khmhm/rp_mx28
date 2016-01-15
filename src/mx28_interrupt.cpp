#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>

#include <wiringPi.h>
#include <wiringSerial.h>

using namespace std;

int fd;


void MX28_write(int ID, int start, int bytes, char* data, int fd)
{
	char TxBuf[16];
	char sum = 0;
	char Status[6];

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
		serialPutchar(fd, TxBuf[i]);
	}

	//for(int i=0; i<15; i++){
	for(int i=0; i<6; i++){
		Status[i] = serialGetchar(fd);
		printf("%d ", Status[i]);
	}
	puts("");
}





void MX28SetMovingSpeed(float rpm, int fd)
{
	short data;
	char byte[2];
	data = (int)(fabs(rpm)/0.1144);
	byte[0] = 0xff & data;
	byte[1] = data >>8;
	if(rpm<0){
		byte[1] |= 0x04;
	}
	MX28_write(0x01, 0x20, 2, byte, fd);
}


float ReadPresentPosition(void)
{
	char TxBuf[16];
	char sum = 0;
	char RxBuf[16];

	TxBuf[0] = 0xff;
	TxBuf[1] = 0xff;
	
	TxBuf[2] = 0x01;
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
	float radian = (RxBuf[5] + RxBuf[6]*256.0) * 2.0 * M_PI / 4096.0 ;
	//if(radian > M_PI){
	//	radian -= 2.0*M_PI;
	//}
	return radian;
	//float degree = (RxBuf[13] + RxBuf[14]*256.0) * 360.0 / 4096.0;
	//return degree;
}	

ros::Publisher position_pub;
geometry_msgs::PoseStamped degree;
void interrupt(void)
{
	//puts("interrupt!");
	//ReadPresentPosition();

	degree.header.stamp = ros::Time::now();
	degree.pose.position.x = ReadPresentPosition();

	//cout<<degree.pose.position.x<<endl;
	
	position_pub.publish(degree);
}

//void signal_handle(int)
//{
//	MX28SetMovingSpeed(0, fd);
//	exit(0);
//}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "mx28_test");
	ros::NodeHandle n;
	//ros::Publisher position_pub = n.advertise<geometry_msgs::PoseStamped>("ang", 1);
	position_pub = n.advertise<geometry_msgs::PoseStamped>("ang", 1);

	fd = serialOpen("/dev/ttyAMA0", 57600);
	if(fd<0){
		printf("can not open serial port");
	}

	printf("%d\n", argc);
	if(argc==2){
		MX28SetMovingSpeed(atof(argv[1]), fd);
	}else {
		MX28SetMovingSpeed(5, fd);
	}

	if(wiringPiSetupSys() == -1){
		puts("wiringPi has not been initialised!");
	}

	wiringPiISR( 26, INT_EDGE_FALLING, interrupt);


	//geometry_msgs::PoseStamped degree;
	
	while(ros::ok()){
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
	}

	MX28SetMovingSpeed(0, fd);

	return 0;
}
