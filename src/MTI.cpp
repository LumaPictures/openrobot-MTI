
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
#include <string>
#include <sstream>

static std::string getHorodatage()
{
  struct timespec Timespec;
  std::ostringstream oss;

  clock_gettime(CLOCK_REALTIME,&Timespec);

  oss << Timespec.tv_sec << "." << Timespec.tv_nsec;
  return oss.str();
}

MTI::MTI(const char * dev_, 
	 OutputMode mode_, 
	 OutputFormat outputDisplay_, 
	 SyncOutMode syncOutMode_):
	device(dev_), mode(MTI_OPMODE_CALIBRATED), outputDisplay(MTI_OPFORMAT_EULER), outputSkipFactor(0), mtcomm(), connected(false),
{
	_set_mode(mode_);
	_set_outputDisplay(outputDisplay_);
	_set_syncOut(syncOutMode_);
	connect();
	_configure_device();
}

MTI::~MTI()
{
	disconnect();
}


bool MTI::connect()
{
	if (!connected) {
		if (mtcomm.openPort(device) != MTRV_OK) {
			std::cerr << "MTI: Can't open " << device << std::endl;
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

	mtcomm.close();
	connected = false;
	return true;
}

bool MTI::_set_mode(OutputMode mode_)
{

	switch (mode_) {
		case MTI_OPMODE_CALIBRATED:
			std::cout << "MTI: Calibrated output." << mode_ << std::endl;
			break;
		case MTI_OPMODE_ORIENTATION:
			std::cout << "MTI: Orientation output." << mode_ << std::endl;
			break;
		case MTI_OPMODE_BOTH:
			std::cout << "MTI: Calibrated and orientation outputs." << mode_ << std::endl;
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

	outputDisplay |= OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	return true;
}

bool MTI::_set_syncOut( SyncOutMode          syncOutMode_          ,
			SyncOutPulsePolarity syncOutPulsePolarity_ ,
			int syncOutSkipFactor_    ,
			int syncOutOffset_        ,
			int syncOutPulseWidth_    )
{
	syncOutMode = syncOutMode_ | syncOutPulsePolarity_;
	
	syncOutSkipFactor = syncOutSkipFactor_;
	
	
	if ((syncOutOffset_ == 0) || (syncOutOffset_ >= 513)) 
		syncOutOffset = syncOutOffset_;
	else syncOutOffset = 0;


	if ((syncOutPulseWidth_ == 0) || (syncOutPulseWidth_ >= 1700)) 
		syncOutPulseWidth = syncOutPulseWidth_;
	else syncOutPulseWidth = 2049;
	
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
	unsigned long tmpMode, tmpOutputDisplay;
	unsigned short tmpDataLength;
	unsigned short numDevices;

	if (!connected)
		return false;

	// Put MTi/MTx in Config State
	if (mtcomm.writeMessage(MID_GOTOCONFIG) != MTRV_OK) {
		std::cerr << "Can't put the device in Config / state" << std::endl;
		return false;
	}

	// Get current settings and check if Xbus Master is connected
	if (mtcomm.getDeviceMode(&numDevices) != MTRV_OK)
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

	// Check if Xbus Master is connected
	mtcomm.getMode(tmpMode, tmpOutputDisplay, tmpDataLength, BID_MASTER);
	if (tmpMode == OUTPUTMODE_XM)
	{
		// If Xbus Master is connected, attached Motion Trackers should not send sample counter
		outputDisplay &= 0xFFFFFFFF - OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	}

	// Set output mode and output settings for each attached MTi/MTx
	for (int i = 0; i < numDevices; i++)
	{
		if (mtcomm.setDeviceMode(mode, outputDisplay, BID_MT + i) != MTRV_OK)
		{
			std::cerr << "Could not set (all) device mode(s)" << std::endl;
			return false;
		}
	}
	
	// Set syncOut Settings for each attached MTi
	for (int i = 0; i<numDevices; i++)
	{
		if (mtcomm.setDeviceSyncOut(syncOutMode, syncOutSkipFactor, syncOutOffset, syncOutPulseWidth, BID_MT + i) != MTRV_OK)
		{
			std::cerr << "Could not set (all) device syncOut setting(s)" << std::endl;
			return false;
		}
	}
	
	// Set outputSkipFactor Settings for each attached MTi
	for (int i = 0; i<numDevices; i++)
	{
		if (mtcomm.setSetting(MID_SETOUTPUTSKIPFACTOR, outputSkipFactor, LEN_OUTPUTSKIPFACTOR, BID_MT + i) != MTRV_OK)
		{
			std::cerr << "Could not set (all) device outputSkipFactor setting(s)" << std::endl;
			return false;
		}
	}

	// Put MTi/MTx in Measurement State
	mtcomm.writeMessage(MID_GOTOMEASUREMENT);

	return true;
}


bool MTI::read(INERTIAL_DATA * output, bool verbose)
{
	if (!connected)
		return false;

	unsigned char data[MAXMSGLEN];
	short datalen;
	float fdata[18] = {0};

	std::string msgHorodatage;

	std::cout.precision(3);

	///////////////////////////////////////////////////
	// read inertial sensor and get data
	//
	///////////////////////////////////////////////////
	if(mtcomm.readDataMessage(data, datalen) == MTRV_OK)
	{
		unsigned short samplecounter;
		// get real time clock
		msgHorodatage = getHorodatage();
		mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);      
		output->COUNT = samplecounter;		 

		if ((mode & OUTPUTMODE_CALIB) != 0)
		{
			// ACC		  
			mtcomm.getValue(VALUE_CALIB_ACC, fdata, data, BID_MT);	
			if (verbose)
				std::cout << "ACC " << msgHorodatage << " " << fdata[0] << " " << fdata[1] << " " << fdata[2] << std::endl;

			output->ACC[0]=fdata[0];
			output->ACC[1]=fdata[1];
			output->ACC[2]=fdata[2];

			// GYR		  		    
			mtcomm.getValue(VALUE_CALIB_GYR, fdata, data, BID_MT);
			if (verbose)
				std::cout << "GYR " << msgHorodatage << " " << fdata[0] << " " << fdata[1] << " " << fdata[2] << std::endl;

			output->GYR[0]=fdata[0];
			output->GYR[1]=fdata[1];
			output->GYR[2]=fdata[2];

			// MAGN
			mtcomm.getValue(VALUE_CALIB_MAG, fdata, data, BID_MT);	
			if (verbose)
				std::cout << "MAG " << msgHorodatage << " " << fdata[0] << " " << fdata[1] << " " << fdata[2] << std::endl;

			output->MAG[0]=fdata[0];
			output->MAG[1]=fdata[1];
			output->MAG[2]=fdata[2];
		}//endif

		if ((mode & OUTPUTMODE_ORIENT) != 0)
		{
			switch(outputDisplay & OUTPUTSETTINGS_ORIENTMODE_MASK)
			{
				case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
					// Output: quaternion
					mtcomm.getValue(VALUE_ORIENT_QUAT, fdata, data, BID_MT);
					if (verbose)
						std::cout << fdata[0] << "\t" << fdata[1] << "\t" << fdata[2] << "\t" << fdata[3] << std::endl;
					break;

				case OUTPUTSETTINGS_ORIENTMODE_EULER:
					// Output: Euler
					msgHorodatage = getHorodatage();
 					mtcomm.getValue(VALUE_ORIENT_EULER, fdata, data, BID_MT);	
					if (verbose)
						std::cout << "EUL " << msgHorodatage << " " << fdata[0] << " " << fdata[1] << " " << fdata[2] << std::endl;

					// lien avec le poster de GENOM
					output->EULER[0]=fdata[0];
					output->EULER[1]=fdata[1];
					output->EULER[2]=fdata[2];		  		  		    
					break;

				case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
					// Output: Cosine Matrix
					mtcomm.getValue(VALUE_ORIENT_MATRIX, fdata, data, BID_MT);
					if (verbose) {
						std::cout << fdata[0] << "\t" << fdata[1] << "\t" << fdata[2] << std::endl;
						std::cout << fdata[3] << "\t" << fdata[4] << "\t" << fdata[5] << std::endl;
						std::cout << fdata[6] << "\t" << fdata[7] << "\t" << fdata[8] << std::endl;
					}
					break;

				default:
					;
			}// end switch
		} // endif	
	} //endif check read data
	else // display error on read buffer
	{
		std::cerr << "MTI failed to read message, code " <<  mtcomm.getLastRetVal() << std::endl;
		return false;
	}

	return true;
}

bool MTI::_reset() {
	if (!connected)
		return false;
	if (mtcomm.writeMessage(MID_RESET) != MTRV_OK) 
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
	if (mtcomm.writeMessage(MID_GOTOCONFIG) != MTRV_OK) {
		std::cerr << "Can't put the device in Config / state" << std::endl;
		return false;
	}

	// Get current settings and check if Xbus Master is connected
	if (mtcomm.getDeviceMode(&numDevices) != MTRV_OK)
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
	if (mtcomm.setSetting(MID_SETBAUDRATE, baudRateMti, LEN_BAUDRATE, BID_MT) != MTRV_OK)
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

