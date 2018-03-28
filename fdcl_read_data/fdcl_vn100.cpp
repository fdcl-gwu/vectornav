#include "fdcl_vn100.h"

fdcl_vn100::~fdcl_vn100()
{
}

fdcl_vn100::fdcl_vn100()
{
}

void fdcl_vn100::open()
{
	port  = "/dev/ttyUSB0";
	baud_rate = 115200;

	readAscii(port, baud_rate);
}

void fdcl_vn100::init(string port, int baud_rate)
{
	// Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
	for (int i = 0; i < 5; i++)
	{
		Thread::sleepMs(200);

		// This reads the latest data that has been processed by the EzAsyncData class.
		CompositeData cd = ez->currentData();

		// Make sure that we have some yaw, pitch, roll data.
		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}
}

void fdcl_vn100::readAscii(string port, const int baud_rate)
{

	// We create and connect to a sensor by the call below.
	ez = EzAsyncData::connect(port, baud_rate);

	init(port, baud_rate);

	// Most of the asynchronous data handling is done by EzAsyncData but there are times
	// when we wish to configure the sensor directly while still having EzAsyncData do
	// most of the grunt work.This is easily accomplished and we show changing the ASCII
	// asynchronous data output type here.
	try
	{
		ez->sensor()->writeAsyncDataOutputType(VNYPR);
	}
	catch (...)
	{
		cout << "Error setting async data output type." << endl;
	}

	cout << "[New ASCII Async Output]" << endl;

	// We can now display yaw, pitch, roll data from the new ASCII asynchronous data type.
	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(200);

		CompositeData cd = ez->currentData();

		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// The CompositeData structure contains some helper methods for getting data
	// into various formats. For example, although the sensor is configured to
	// output yaw, pitch, roll, our application might need it as a quaternion
	// value. However, if we query the quaternion field, we see that we don't
	// have any data.

	cout << "HasQuaternion: " << ez->currentData().hasQuaternion() << endl;

	// Uncommenting the line below will cause an exception to be thrown since
	// quaternion data is not available.

	// cout << "Current Quaternion: " << ez->currentData().quaternion() << endl;

	// However, the CompositeData structure provides the anyAttitude field
	// which will perform the necessary conversions automatically.

	cout << "[Quaternion from AnyAttitude]" << endl;

	for (int i = 0; i < 25; i++)
	{
		Thread::sleepMs(200);

		// This reads the latest data that has been processed by the EzAsyncData class.
		CompositeData cd = ez->currentData();

		// Make sure that we have some attitude data.
		if (!cd.hasAnyAttitude())
		cout << "Attitude Unavailable." << endl;
		else
		cout << "Current Quaternion: " << cd.anyAttitude().quat() << endl;
	}

	// Throughout this example, we have been using the ez->currentData() to get the most
	// up-to-date readings from the sensor that have been processed. When called, this
	// method returns immediately with the current values, thus the reason we have to
	// put the Thread::sleepMs(200) in the for loop. Otherwise, we would blaze through
	// the for loop and just print out the same values. The for loop below illustrates
	// this.

	cout << "[For Loop Without Sleep]" << endl;

	for (int i = 0; i < 25; i++)
	{
		CompositeData cd = ez->currentData();

		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	// Often, we would like to get and process each packet received from the sensor.
	// This is not realistic with ez->currentData() since it is non-blocking and we
	// would also have to compare each CompositeData struture for changes in the data.
	// However, EzAsyncData also provides the getNextData() method which blocks until
	// a new data packet is available. The for loop below shows how to output each
	// data packet received from the sensor using getNextData().

	cout << "[getNextData Method]" << endl;

	for (int i = 0; i < 25; i++)
	{
		CompositeData cd = ez->getNextData();

		if (!cd.hasYawPitchRoll())
		cout << "YPR Unavailable." << endl;
		else
		cout << "Current YPR: " << cd.yawPitchRoll() << endl;
	}

	ez->disconnect();

	delete ez;

}
