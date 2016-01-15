/************************************************
*
*	mx28.h
*	by 小沼
*	
*	サーボモータmx28をRaspberryPi（１でも２でも）
*	を使って動かす
*
*	* RaspberryPiとMX28の接続方法
*		RaspberryPiのUARTをrs485レベルに変換して
*		MX28と接続する
*		詳細はMX28で検索したりしてください
*	* RaspberryPiのOSはUBUNTU14.04でやったが，
*	  raspbianでも多分大丈夫
*
*
*
*
*
*
*
************************************************/


#include <stdio.h>
#include <stdlib.h>

class MX28{
public:
	MX28(const char* port_name, int id, int baud_rate);
