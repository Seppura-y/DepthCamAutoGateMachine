#include "frame_proc_thread.h"

#include<iostream>
#include <thread>

using namespace std;

typedef struct AreaStatus {
	uint8_t reserve1 : 1;
	uint8_t reserve2 : 1;
	uint8_t reserve3 : 1;
	uint8_t reserve4 : 1;
	uint8_t reserve5 : 1;
	uint8_t reserve6 : 1;
	uint8_t reserve7 : 1;
	uint8_t reserve8 : 1;
} AreaStatus_t;

int main(int argc, char** argv)
{
	//printf(out.str().c_str());
	//uint8_t tmp = ~0;
	//tmp = tmp << 2;

	AreaStatus sta = { 0 };
	sta.reserve1 = 1;
	sta.reserve2 = 2;

	

	FrameProcThread th;
	th.GetDeviceCount();
	th.SetDeviceIndex(0);
	th.Start();

	while (true)
	{
		this_thread::sleep_for(1ms);
	}
	th.Exit();
	return 0;
}
