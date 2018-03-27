#include <iostream>

// Include this header file to get access to the EzAsyncData class.
#include "vn/ezasyncdata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// #include "vectornav.h"
// #include "fdcl_param.h"
// #include "misc_matrix_func.h"
//
class fdcl_vn100
{
public:
	fdcl_vn100();
	// 	~fdcl_vn100();
	//
	// 	string port; //"/dev/ttyTHS2";
	// 	int baud_rate;
	//
	// 	static void callback(void* sender, VnDeviceCompositeData* data);
	//
	// 	void load_config(fdcl_param& );
	// 	void open();
	// 	void open(string port, const int baud_rate);
	// 	void close();
	//
	// private:
	// 	VN_ERROR_CODE errorCode;
	// 	Vn100 vn100;
};
