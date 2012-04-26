
/*!
 * Copyright (c) 2008 Arnaud Degroote
 * Copyright (c) 2009 Joan Sola
 *
 * $Source:
 * $Revision: 0.d
 * $Date: 17 August 2009
 *
 */

#include "MTI/MTI.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#ifdef __APPLE__
#	include <mach/clock.h>
#	include <mach/mach.h>
#endif

using namespace xsens;

#define reportXsensResultErr(msg, resultObj) { std::cerr << (msg) << " - code " << static_cast<int>((resultObj)) << ": " << xsensResultText((resultObj)) << std::endl; }

static double getHorodatage()
{
	struct timespec ts;

#ifdef __APPLE__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	ts.tv_sec = mts.tv_sec;
	ts.tv_nsec = mts.tv_nsec;
#else // __APPLE__
	 clock_gettime(CLOCK_REALTIME,&ts);
#endif // __APPLE__

	return ts.tv_sec + ts.tv_nsec*1e-9;
}



static std::string convertHorodatageToString(double t)
{
	std::ostringstream oss;
	oss << std::setprecision(19) << t;
	return oss.str();
}


MTI::MTI(const char * dev_,
	 OutputMode mode_,
	 OutputFormat outputDisplay_,
	 SyncOutMode syncOutMode_):
	device(dev_), mode(MTI_OPMODE_CALIBRATED), outputDisplay(MTI_OPFORMAT_EULER), outputSkipFactor(0), mtcomm(), connected(false),
	pte(10e-3, 6000, 0.0002)
{
	baudrate_enum = CMT_BAUD_RATE_115K2;
	baudrate = 115200;

	_set_mode(mode_);
	_set_outputDisplay(outputDisplay_);
	_set_syncOut(syncOutMode_);
	connect();
	_configure_device();

	// old version of mtsdk didn't have separate measurement / default timeout;
	// so for backward compatibility, set the measurement timeout to set the
	// global timeout
	mtcomm.setTimeoutMeasurement(CMT_TO_DEFAULT);
}

MTI::~MTI()
{
	disconnect();
}


bool MTI::connect()
{
	if (!connected) {
		XsensResultValue result = mtcomm.openPort(device, baudrate_enum);
		if (result != XRV_OK) {
			std::ostringstream oss;
			oss << "MTI: Can't open " << device;
			reportXsensResultErr(oss.str(), result);
			return false;
		} else {
			std::cerr << "MTI: Open device " << device << std::endl;
			connected = true;
		}
	}

	return true;
}

bool MTI::connect(const char * dev)
{
	device = dev;
	connect();
	return _configure_device();
}


bool MTI::disconnect()
{
	if (!connected)
		return true;

	mtcomm.closePort(true);
	mtcomm.closeLogFile();
	connected = false;
	return true;
}

bool MTI::_set_mode(OutputMode mode_)
{

	switch (mode_) {
		case MTI_OPMODE_CALIBRATED:
			std::cout << "MTI: Calibrated output." << mode_ << std::endl;
			timestamp_delay = 0.31e-3; // cf section 5.4 of MTi and MTx User Manual and Technical Documentation
			break;
		case MTI_OPMODE_ORIENTATION:
			std::cout << "MTI: Orientation output." << mode_ << std::endl;
			timestamp_delay = 2.55e-3; // cf section 5.4 of MTi and MTx User Manual and Technical Documentation
			break;
		case MTI_OPMODE_BOTH:
			std::cout << "MTI: Calibrated and orientation outputs." << mode_ << std::endl;
			timestamp_delay = 2.55e-3; // cf section 5.4 of MTi and MTx User Manual and Technical Documentation
			break;
		default:
			return false;
	}
	mode = mode_;
	return true;
}

bool MTI::_set_outputDisplay(OutputFormat outputDisplay_)
{
	switch(outputDisplay_) {
		case MTI_OPFORMAT_QUAT:
			std::cout << "MTI: Quaternion mode" << std::endl;
			break;
		case MTI_OPFORMAT_EULER:
			std::cout << "MTI: Euler angles mode" << std::endl;
			break;
		case MTI_OPFORMAT_MAT:
			std::cout << "MTI: Rotation matrix mode" << std::endl;
			break;
	}

	outputDisplay = outputDisplay_;

	outputDisplay |= CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	return true;
}

bool MTI::_set_syncOut( SyncOutMode          syncOutMode_          ,
			SyncOutPulsePolarity syncOutPulsePolarity_ ,
			int syncOutSkipFactor_    ,
			int syncOutOffset_        ,
			int syncOutPulseWidth_    )
{
	syncOut.m_mode = syncOutMode_ | syncOutPulsePolarity_;
	syncOut.m_skipFactor = syncOutSkipFactor_;

	if ((syncOutOffset_ == 0) || (syncOutOffset_ >= 513))
		syncOut.m_offset = syncOutOffset_;
	else syncOut.m_offset = 0;

	if ((syncOutPulseWidth_ == 0) || (syncOutPulseWidth_ >= 1700))
		syncOut.m_pulseWidth = syncOutPulseWidth_;
	else syncOut.m_pulseWidth = 2049;

	return true;
}

bool MTI::_set_outputSkipFactor(int factor)
{
	if (factor < 0) return false;
	outputSkipFactor = factor;
	return true;
}


bool MTI::set_syncOut(  SyncOutMode syncOutMode_          ,
			SyncOutPulsePolarity syncOutPulsePolarity_ ,
			int syncOutSkipFactor_    ,
			int syncOutOffset_        ,
			int syncOutPulseWidth_    )
{
	return (_set_syncOut(syncOutMode_,
			     syncOutPulsePolarity_ ,
			     syncOutSkipFactor_    ,
			     syncOutOffset_        ,
			     syncOutPulseWidth_    ) && _configure_device());
}


bool MTI::set_mode(OutputMode mode_)
{
	return (_set_mode(mode_) && _configure_device());
}

bool MTI::set_outputDisplay(OutputFormat outputDisplay_)
{
	return (_set_outputDisplay(outputDisplay_) && _configure_device());
}

bool MTI::set_outputSkipFactor(int factor)
{
	return (_set_outputSkipFactor(factor) && _configure_device());
}


bool MTI::_configure_device()
{
	CmtDeviceMode2 tmpMode;
	unsigned short numDevices;
	XsensResultValue result;

	if (!connected)
		return false;

	// Put MTi/MTx in Config State
	result = mtcomm.gotoConfig();
	if (result != XRV_OK) {
		reportXsensResultErr("Can't put the device in Config / state", result);
		return false;
	}

	// Get current settings and check if Xbus Master is connected
	result = mtcomm.refreshCache();
	numDevices = mtcomm.getDeviceCount();
	if (result != XRV_OK)
	{
		if (numDevices == 1) {
			reportXsensResultErr("Can't put the device in Config / state", result);
			std::cerr << "Could not get device mode" << std::endl;
		} else {
			std::cerr << "Not just MTi / MTx connected to Xbus" << std::endl;
			reportXsensResultErr("Could not get all devices modes", result);
		}

		return false;
	}

	if (mtcomm.isXm())
	{
		// If Xbus Master is connected, attached Motion Trackers should not send sample counter
		outputDisplay &= 0xFFFFFFFF - CMT_OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	}

	// Set output mode and output settings for each attached MTi/MTx
	tmpMode.m_outputMode = mode;
	tmpMode.m_outputSettings = outputDisplay;
	tmpMode.m_skip = outputSkipFactor;
	result = mtcomm.setDeviceMode2(tmpMode, false, CMT_DID_BROADCAST);
	if(result != XRV_OK)
	{
		reportXsensResultErr("Could not set (all) device mode(s)", result);
		return false;
	}

	// Set syncOut Settings
	result = _configure_syncOut();
	if(result != XRV_OK)
	{
		reportXsensResultErr("Could not set (all) device syncOut setting(s)", result);
		return false;
	}

	// Put MTi/MTx in Measurement State
	mtcomm.gotoMeasurement();

	return true;
}

XsensResultValue MTI::_configure_syncOut()
{
	if (mtcomm.isXm())
	{
		return mtcomm.setSyncOutSettings(syncOut);
	}
	else
	{
		XsensResultValue result;
		// Sadly, cmt3 doesn't include a built in way to set all syncOut settings
		// for individual MT devices, only for the xbus... so we need to set
		// all 4
		mtcomm.setSyncOutMode(syncOut.m_mode);
		mtcomm.setSyncOutOffset(syncOut.m_offset);
		mtcomm.setSyncOutPulseWidth(syncOut.m_pulseWidth);
		mtcomm.setSyncOutSkipFactor(syncOut.m_skipFactor);
	}

}

bool MTI::read(INERTIAL_DATA * output, bool verbose)
{
	if (!connected)
		return false;

	Packet reply(mtcomm.getDeviceCount(), false);

	std::string msgHorodatage;

	std::cout.precision(3);

	///////////////////////////////////////////////////
	// read inertial sensor and get data
	//
	///////////////////////////////////////////////////
	double date;
	int res;
	if((res = mtcomm.waitForDataMessage(&reply)) == XRV_OK)
	{
		// get real time clock
		date = getHorodatage();

		output->TIMESTAMP_RAW = date;

		uint16_t samplecounter = reply.getSampleCounter();

		double delay = timestamp_delay + reply.m_msg.getTotalMessageSize()/baudrate;
		date -= delay;
		output->TIMESTAMP_UNDELAYED = date;

		int nperiods = samplecounter - output->COUNT;
		if (nperiods < 0) nperiods += 65536;
		date = pte.estimate(output->TIMESTAMP_RAW, nperiods, delay);

		output->COUNT = samplecounter;
		output->TIMESTAMP_FILTERED = date;

		output->TIMESTAMP = output->TIMESTAMP_UNDELAYED;
		msgHorodatage = convertHorodatageToString(output->TIMESTAMP);

		if (verbose)
			std::cout << msgHorodatage;

		if (reply.containsCalData())
		{
			CmtVector data;

			// ACC
			data = reply.getCalAcc();
			if (verbose)
				std::cout << "\tACC\t" << data.m_data[0] << "\t" << data.m_data[1] << "\t" << data.m_data[2];

			output->ACC[0]=data.m_data[0];
			output->ACC[1]=data.m_data[1];
			output->ACC[2]=data.m_data[2];

			// GYR
			data = reply.getCalGyr();
			if (verbose)
				std::cout << "\tGYR\t" << data.m_data[0] << "\t" << data.m_data[1] << "\t" << data.m_data[2];

			output->GYR[0]=data.m_data[0];
			output->GYR[1]=data.m_data[1];
			output->GYR[2]=data.m_data[2];

			// MAGN
			data = reply.getCalMag();
			if (verbose)
				std::cout << "\tMAG\t" << data.m_data[0] << "\t" << data.m_data[1] << "\t" << data.m_data[2];

			output->MAG[0]=data.m_data[0];
			output->MAG[1]=data.m_data[1];
			output->MAG[2]=data.m_data[2];
		}//endif

		if (reply.containsOri())
		{
			if (reply.containsOriQuat())
			{
				// Output: quaternion
				CmtQuat quat = reply.getOriQuat();
				if (verbose)
					std::cout << "\tQUAT\t" << quat.m_data[0] << "\t" << quat.m_data[1] << "\t" << quat.m_data[2] << "\t" << quat.m_data[3];
			}
			if (reply.containsOriEuler())
			{
				CmtEuler euler = reply.getOriEuler();
				if (verbose)
					std::cout << "\tEUL\t" << euler.m_roll << "\t" << euler.m_pitch << "\t" << euler.m_yaw;

				// lien avec le poster de GENOM
				output->EULER[0]=euler.m_roll;
				output->EULER[1]=euler.m_pitch;
				output->EULER[2]=euler.m_yaw;

			}
			if (reply.containsOriMatrix())
			{
				CmtMatrix matrix = reply.getOriMatrix();
				if (verbose) {
					std::cout << "\tMAT\t" << matrix.m_data[0][0] << "\t" << matrix.m_data[0][1] << "\t" << matrix.m_data[0][2]
						<< "\t" << matrix.m_data[1][0] << "\t" << matrix.m_data[1][1] << "\t" << matrix.m_data[1][2]
						<< "\t" << matrix.m_data[2][0] << "\t" << matrix.m_data[2][1] << "\t" << matrix.m_data[2][2];
				}
			}
		} // endif
		if (verbose)
			std::cout << std::endl;
	} //endif check read data
	else // display error on read buffer
	{
		XsensResultValue result = mtcomm.getLastResult();
		reportXsensResultErr("MTI failed to read message", result);
		return false;
	}

	return true;
}

bool MTI::_reset() {
	if (!connected)
		return false;
	if (mtcomm.reset() != XRV_OK)
		return false;

	return true;
}

bool MTI::reset() {
	return _reset();
}

/*bool MTI::_set_baudRate(int baudRate00_)
{
	if (_get_baudRateCodes(baudRate00_) == false)
		return false;
	baudRate = 100*baudRate00_;
}

bool MTI::_configure_baudRate()
{
	if (!connected)
		return false;

	// Put MTi/MTx in Config State
	if (mtcomm.writeMessage(MID_GOTOCONFIG) != XRV_OK) {
		std::cerr << "Can't put the device in Config / state" << std::endl;
		return false;
	}

	// Get current settings and check if Xbus Master is connected
	if (mtcomm.getDeviceMode(&numDevices) != XRV_OK)
	{
		if (numDevices == 1) {
			std::cerr << "MTI / MTx has not been detected" << std::endl;
			std::cerr << "Could not get device mode" << std::endl;
		} else {
			std::cerr << "Not just MTi / MTx connected to Xbus" << std::endl;
			std::cerr << "Could not get all devices modes" << std::endl;
		}

		return false;
	}

	// send baud rate
	if (mtcomm.setSetting(MID_SETBAUDRATE, baudRateMti, LEN_BAUDRATE, BID_MT) != XRV_OK)
	{
	} else {
	}

	// send reset signal

	// Put MTi/MTx in Measurement State
	mtcomm.writeMessage(MID_GOTOMEASUREMENT);

	return true;

}*/

/*bool MTI::_get_baudRateCodes(int baudRate00_)
{
	switch (baudRate00_) {
//		case 48:
//			baudRatePort = PBR_4K8;
//			break;
		case 96:
			baudRatePort = PBR_9600;
			baudRateMti = BAUDRATE_9K6;
			break;
//		case 144:
//			baudRatePort = PBR_14K4;
//			baudRateMti = BAUDRATE_14K4;
//			break;
		case 192:
			baudRatePort = PBR_19K2;
			baudRateMti = BAUDRATE_19K2;
			break;
//		case 288:
//			baudRatePort = PBR_28K8;
//			baudRateMti = BAUDRATE_28K8;
//			break;
		case 384:
			baudRatePort = PBR_38K4;
			baudRateMti = BAUDRATE_38K4;
			break;
		case 576:
			baudRatePort = PBR_57K6;
			baudRateMti = BAUDRATE_57K6;
			break;
		case 1152:
			baudRatePort = PBR_115K2;
			baudRateMti = BAUDRATE_115K2;
			break;
		case 2304:
			baudRatePort = PBR_230K4;
			baudRateMti = BAUDRATE_230K4;
			break;
		case 4608:
			baudRatePort = PBR_460K8;
			baudRateMti = BAUDRATE_460K8;
			break;
//		case 9216:
//			baudRatePort = PBR_921K6;
//			break;
		default:
			baudRatePort = -1;
			std::cerr << "Incorrect baudrate." << std::endl;
			return false;
	}
	return true;
}*/


/*bool MTI::connect(const char * dev, int baudRate00_)
{
	device = dev;
	if (_get_baudRateCodes(baudRate00_) == false)
		return false;
	baudRate = 100*baudRate00_;
	connect();
	return _configure_device();
}*/

/*bool MTI::connect(int baudRate00_)
{
	if (_get_baudRateCodes(baudRate00_) == false)
		return false;
	baudRate = 100*baudRate00_;
	return connect();
}*/

