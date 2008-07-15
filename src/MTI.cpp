
/*!
 * Copyright (c) 2008 Arnaud Degroote
 *
 * $Source:
 * $Revision: 0.c
 * $Date: 15 July 2008
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

MTI::MTI(const char * dev_, int mode_, int outputDisplay_):
	device(dev_), mode(0), outputDisplay(0), mtcomm(), connected(false) 
{
	_set_mode(mode_);
	_set_outputDisplay(outputDisplay_);
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
			std::cerr << "Can't open " << device << std::endl;
			return false;
		} else {
			connected = true;
		}
	}

	return true;
}

bool MTI::connect(const char * dev)
{
	device = dev_;
	connect();
	_configure_device();
}

bool MTI::disconnect()
{
	if (!connected)
		return true;

	mtcomm.close();
	connected = false;
	return true;
}

bool MTI::_set_mode(int mode_)
{
	if (mode_ < 1 || mode_ > 3)
		return false;

// mode :
// 1 - Calibrated data
// 2 - Orientation data;
// 3 - Both Calibrated and Orientation data\n

	mode = mode_;
// Update outputMode to match data specs of SetOutputMode
	mode *= 2;

	return true;
}

bool MTI::_set_outputDisplay(int outputDisplay_)
{
	if (outputDisplay_ < 1 || outputDisplay_ > 3)
		return false;
// outputDisplay :
// 1 - Quaternions
// 2 - Euler angles
// 3 - Matrix
	switch(outputDisplay_) {
		case 1:
			outputDisplay= OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
			std::cout << "angle quaternion" << std::endl;
			break;
		case 2:
			outputDisplay= OUTPUTSETTINGS_ORIENTMODE_EULER;
			std::cout << "angle euler" << std::endl;
			break;
		case 3:
			outputDisplay= OUTPUTSETTINGS_ORIENTMODE_MATRIX;
			std::cout << "angle matrix" << std::endl;
			break;
	}

	outputDisplay |= OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
	return true;
}

bool MTI::set_mode(int mode_)
{
	return (_set_mode(mode_) && _configure_device());
}

bool MTI::set_outputDisplay(int outputDisplay_)
{
	return (_set_outputDisplay(outputDisplay_) && _configure_device());
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
