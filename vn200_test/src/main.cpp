#include <iomanip>
#include <iostream>
#include <limits>


// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"

// We need this file for our sleep function.
#include "vn/thread.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// Method declarations for future use.
void asciiAsyncMessageReceived(void* userData, Packet& p, size_t index);
void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

unsigned createMask(unsigned a, unsigned b)
{
	unsigned r = 0;
	for (unsigned i = a; i <= b; i++)
		r |= 1 << i;

	return r;
}

int main(int argc, char *argv[])
{
	// This example walks through using the VectorNav C++ Library to connect to
	// and interact with a VectorNav sensor.

	// First determine which COM port your sensor is attached to and update the
	// constant below. Also, if you have changed your sensor from the factory
	// default baudrate of 115200, you will need to update the baudrate
	// constant below as well.
	// const string port = "COM1";                             // Windows format for physical and virtual (USB) serial port.
	// const string port = "/dev/ttyS1";                    // Linux format for physical serial port.
	// const string port = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
	const string port = "/dev/tty.usbserial-A904RS9S"; // Mac OS X format for virtual (USB) serial port.
	// const string portm = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
	const uint32_t baud_rate = 115200;

	VnSensor vs;
	cout << "Connecting to IMU at " << port << " .." << endl;
	vs.connect(port, baud_rate);

	string mn = vs.readModelNumber();
	cout << "Model Number: " << mn << endl;

	// Get some orientation data from the sensor.
	vec3f ypr = vs.readYawPitchRoll();
	cout << "Current YPR: " << ypr << endl;

	cout << "Setting up binary output register .." << endl;
	vs.writeAsyncDataOutputType(VNOFF); // disable ASCII output

	// Unregister our callback method.
	// vs.unregisterAsyncPacketReceivedHandler();

	BinaryOutputRegister bor(
		ASYNCMODE_PORT2,
		80, // 10 Hz
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELECEF | \
		INSGROUP_LINEARACCELECEF | INSGROUP_POSU | INSGROUP_VELU);
	vs.writeBinaryOutput1(bor);

	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);
	
	cout << "Starting sleep..." << endl;
	Thread::sleepSec(60);
	
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();

	return 0;
}


void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
	{
		vec3f ypr;
		p.parseVNYPR(&ypr);
		cout << "ASCII Async YPR: " << ypr << endl;
		return;
	}

	if (p.type() == Packet::TYPE_BINARY)
	{
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_INSSTATUS | INSGROUP_POSECEF | INSGROUP_VELECEF | 
			INSGROUP_LINEARACCELECEF | INSGROUP_POSU | INSGROUP_VELU))
			// Not the type of binary packet we are expecting.
			{
std::cout << 	1 <<std::endl;
				return;
			}

		// Ok, we have our expected binary output packet. Since there are many
		// ways to configure the binary data output, the burden is on the user
		// to correctly parse the binary packet. However, we can make use of
		// the parsing convenience methods provided by the Packet structure.
		// When using these convenience methods, you have to extract them in
		// the order they are organized in the binary packet per the User Manual.
		uint64_t timeStartup = p.extractUint64();
		vec3f ypr = p.extractVec3f();
		uint16_t ins_status = p.extractUint16();

		unsigned r;

		// INS Status
		// 0 - Not tracking, initializing
		// 1 - Aligning
		// 2 - Tracking
		// 3 - Loss of GPS for more than 45 seconds
		r = createMask(0, 1);
		unsigned int mode = r & ins_status;


		// GPS Fix
		// 0 - No fix
		// 1 - Only UTC
		// 2 - 2D fix
		// 3 - 3D fix
		r = createMask(2, 2);
		unsigned int gps_fix = r & ins_status;


		r = createMask(3, 6);
		unsigned int sensor_error = r & ins_status;

		cout << timeStartup;
		cout << "," << ypr[0] << "," << ypr[1] << "," << ypr[2];
		cout << "," << mode << "," << gps_fix << "," << sensor_error;

		bool flag_ins_available = !(mode==0);
		if (flag_ins_available)
		{
			vec3d pos_ecef = p.extractVec3d();
			vec3f vel_ecef = p.extractVec3f();
			vec3f acc_ecef = p.extractVec3f();
			float posu_ecef = p.extractFloat();
			float velu_ecef = p.extractFloat();

			cout << "," << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << pos_ecef[0] << "," << pos_ecef[1] << "," << pos_ecef[2];
			cout << "," << vel_ecef[0] << "," << vel_ecef[1] << "," << vel_ecef[2];
			cout << "," << acc_ecef[0] << "," << acc_ecef[1] << "," << acc_ecef[2];
			cout << "," << posu_ecef << "," << velu_ecef;
		}

		cout << endl;
	}
}
