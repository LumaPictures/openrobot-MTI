
/*!
 * Copyright (c) 2006 Frederic Camps
 *
 * $Source:
 * $Revision: 0.a
 * $Date: 26 september 2006
 * 
 */

#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include <stdlib.h>

#include "MTI.h"
#include "MTComm.h"

#include <stdio.h>
#include <errno.h>

#include <time.h>
#include <sys/time.h>


CMTComm mtcomm;
int portNumber;
char deviceName[15];
int outputMode;
int outputSettings;
unsigned short numDevices;
int screenSensorOffset = 0;


/*! 
  @brief : Utilisation du programme principal
*/
static void usage(const char *progname)
{
  fprintf(stderr, "usage: %s [-v] serial_device [-l] logFile [-o] mode [-d] mode display\n", progname);
  exit(1);
}

/*! 
  @brief
 
*/
FILE* setLogData(char *_fileName)
{ 
  FILE *fdLog = NULL;
   
  if(_fileName != NULL)
    {
      if ((fdLog=fopen(_fileName, "w+" ))== NULL) 
	{
	  fprintf(stderr, "open log :%s\n", strerror(errno));
	  return NULL;
	}
    }
  return fdLog;
}


/*!
  @brief
*/
void logTrame(FILE *fd, int modeVerbose, char *message)
{
  // verbose and log data into a file
  if(fd != NULL && modeVerbose == 1)
    {
      fprintf(stdout, "%s\n", message);
      fprintf(fd, "%s\n", message);
    }
  // only verbose stdout
  else if(fd == NULL && modeVerbose == 1)
    {
      fprintf(stdout, "%s\n", message);
    }
  // only log file
  else if(fd != NULL && modeVerbose == 0)
    {
      fprintf(fd, "%s\n", message);      
    }  
  fflush(fd);
}



/*! 
  @brief

  @param struct timeval timev : affiche le contenu
*/
void printHorodatage(char *msg)
{    
  struct timespec Timespec;

  clock_gettime(CLOCK_REALTIME,&Timespec);
 
  /* Print the formatted time, in seconds, followed by a decimal point
     and the ns. */
  sprintf (msg,"%ld.%ld", long(Timespec.tv_sec), Timespec.tv_nsec);
}



/*!
  @brief : user setting
*/

void getUserInputs(char *device, int mode, int outputDisplay)
{
#ifdef WIN32
  printf("Enter COM port: ");
  scanf("%d", &portNumber);
#else
  strcpy(deviceName, device);
#endif
	

// mode :
// 1 - Calibrated data
// 2 - Orientation data\n");
// 3 - Both Calibrated and Orientation data\n

 outputMode = mode;

// outputDisplay :
// 1 - Quaternions
// 2 - Euler angles
// 3 - Matrix

// Update outputMode to match data specs of SetOutputMode
outputMode <<= 1;

  switch(outputDisplay) {
	case 1:
		outputSettings = OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
		printf("angle quaternion\n"); 
		break;
	case 2:
		outputSettings = OUTPUTSETTINGS_ORIENTMODE_EULER;
		 printf("angle euler\n"); 
		break;
	case 3:
		outputSettings = OUTPUTSETTINGS_ORIENTMODE_MATRIX;
 		printf("angle matrix\n"); 
		break;
	}

  outputSettings |= OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
}



/*!
  @brief : recording user settings into inertial sensor befor measurement
*/
int doMtSettings(void) 
{
  unsigned long tmpOutputMode, tmpOutputSettings;
  unsigned short tmpDataLength;

  // Put MTi/MTx in Config State
  if(mtcomm.writeMessage(MID_GOTOCONFIG) != MTRV_OK)
    {
      printf("No device connected\n");
      return -1;
    }

  // Get current settings and check if Xbus Master is connected
  if (mtcomm.getDeviceMode(&numDevices) != MTRV_OK)
    {
      if (numDevices == 1)
	printf("MTi / MTx has not been detected\nCould not get device mode\n");
      else
	printf("Not just MTi / MTx connected to Xbus\nCould not get all device modes\n");
      return -1;
    }
	
  // Check if Xbus Master is connected
  mtcomm.getMode(tmpOutputMode, tmpOutputSettings, tmpDataLength, BID_MASTER);
  if (tmpOutputMode == OUTPUTMODE_XM)
    {
      // If Xbus Master is connected, attached Motion Trackers should not send sample counter
      outputSettings &= 0xFFFFFFFF - OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
    }
	
  // Set output mode and output settings for each attached MTi/MTx
  for (int i = 0; i < numDevices; i++)
    {
      if (mtcomm.setDeviceMode(outputMode, outputSettings, BID_MT + i) != MTRV_OK)
	{
	  printf("Could not set (all) device mode(s)\n");
	  return -1;
	}
    }

  // Put MTi/MTx in Measurement State
  mtcomm.writeMessage(MID_GOTOMEASUREMENT);

  return 1;
}


/*! 
  @brief Initialisation de la centrale par un programme externe - > module Genome
  

 @param 
*/
int initInertialSensor()
{ 
  unsigned char data[MAXMSGLEN];
  short datalen;
  float fdata[18] = {0};
  unsigned short samplecounter;

  // convert float to string to log
  char msg[250];
  char msgHorodatage[100];
  static char *logFile=NULL;
  int verbose = 1;
  FILE *fdLog = NULL;


  // load user settings in inertial sensor
  //getUserInputs(device, mode, outputDisplay);	

  // Open and initialize serial port
#ifdef WIN32
  if (mtcomm.openPort(portNumber) != MTRV_OK)
    {
      printf("Cannot open COM port %s mode: %d\n", device, mode);
#else
      if (mtcomm.openPort(deviceName) != MTRV_OK)
	{
	  printf("MTI Cannot open COM port %s\n", deviceName);
#endif
	  return MTRV_INPUTCANNOTBEOPENED;
	}	

      if(doMtSettings() == -1)
	return MTRV_UNEXPECTEDMSG;
       
      // output format logged into log file if needed
      
      memset(msg, 0, 50);
      sprintf(msg,"MTI Calibrated sensor data - LAAS/CNRS 2007\n");
      logTrame(fdLog, verbose, msg);
      memset(msg, 0, 50);
      sprintf(msg,"ACCX ACCY ACCZ : unity m/s2 \nGYRX GYRY GYRZ : unity rad/s\nMAGNX MAGNY MAGNZ :\
       arbitrary units normalized to earth field strength\nEulerX EulerY EulerZ : unity degree\n");   
      // initial value for inertial sensor
      logTrame(fdLog, verbose, msg);
	
      ///////////////////////////////////////////////////
      // read inertial sensor and get data
      //
      ///////////////////////////////////////////////////
      if(mtcomm.readDataMessage(data, datalen) == MTRV_OK)
	{
	  // get real time clock
	  printHorodatage(msgHorodatage);

	  mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);      		 
	 				
	  if ((outputMode & OUTPUTMODE_CALIB) != 0)
	    {
	      // Output Calibrated data
	      mtcomm.getValue(VALUE_CALIB_ACC, fdata, data, BID_MT);	
		      	      
	      // lien avec le poster de GENOM
	      _ACC[0]=fdata[0];
	      _ACC[1]=fdata[1];
	      _ACC[2]=fdata[2];

	      // ACC		  
	      memset(msg, 0, 50);
	      sprintf(msg,"ACC %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);		     
	      logTrame(fdLog, verbose, msg);		   					
	      mtcomm.getValue(VALUE_CALIB_GYR, fdata, data, BID_MT);


		  
	      // GYR		  		    
	      memset(msg, 0, 50);
	      sprintf(msg,"GYR %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
	      logTrame(fdLog, verbose, msg);		    					 
	      mtcomm.getValue(VALUE_CALIB_MAG, fdata, data, BID_MT);	
	      
	       // lien avec le poster de GENOM
	      _GYR[0]=fdata[0];
	      _GYR[1]=fdata[1];
	      _GYR[2]=fdata[2];
	
		  
	      // MAGN
	      memset(msg, 0, 50);
	      sprintf(msg,"MAGN %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
	      logTrame(fdLog, verbose, msg);

	      // lien avec le poster de GENOM
	      _MAG[0]=fdata[0];
	      _MAG[1]=fdata[1];
	      _MAG[2]=fdata[2];

		  
	    }//endif

	  if ((outputMode & OUTPUTMODE_ORIENT) != 0)
	    {
	      switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK)
		{
		case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
		  // Output: quaternion
		  mtcomm.getValue(VALUE_ORIENT_QUAT, fdata, data, BID_MT);
		  printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n",
			 fdata[0],
			 fdata[1], 
			 fdata[2], 
			 fdata[3]); 
		  break;

		case OUTPUTSETTINGS_ORIENTMODE_EULER:
		  // Output: Euler
		  printHorodatage(msgHorodatage);
		  memset(msg, 0, 50);
		  mtcomm.getValue(VALUE_ORIENT_EULER, fdata, data, BID_MT);	

		  // lien avec le poster de GENOM
		  _EULER[0]=fdata[0];
		  _EULER[1]=fdata[1];
		  _EULER[2]=fdata[2];		  		  		    
		  		    			  
		  break;

		case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
		  // Output: Cosine Matrix
		  mtcomm.getValue(VALUE_ORIENT_MATRIX, fdata, data, BID_MT);
		  printf("%6.3f\t%6.3f\t%6.3f\n",fdata[0], 
			 fdata[1], 
			 fdata[2]);
		  printf("%6.3f\t%6.3f\t%6.3f\n",fdata[3],
			 fdata[4], 
			 fdata[5]);
		  printf("%6.3f\t%6.3f\t%6.3f\n",fdata[6], 
			 fdata[7], 
			 fdata[8]);
		  break;

		default:
		  ;
		}// end switch
	    } // endif	
	} //endif check read data
      else // display error on read buffer
	{
	  if(logFile != NULL)
	    {
	      memset(msg, 0, 50);	
	      sprintf(msg, "failed to read message, code (%d)\n", mtcomm.getLastRetVal());
	      logTrame(fdLog, verbose, msg);
	    }
	  else
	    fprintf(stderr, "MTI failed to read message, code (%d)\n", mtcomm.getLastRetVal());
	}

      // When done, close the serial port
      mtcomm.close();

      return MTRV_OK;

    }


/*!
  @brief : main API
*/
int startMTI(int argc, char *argv[]) 
{
  unsigned char data[MAXMSGLEN];
  short datalen;
  float fdata[18] = {0};
  unsigned short samplecounter;

  // convert float to string to log
  char msg[250];
  char msgHorodatage[100];

  int  ch;
  char *prog = argv[0];
  static char *logFile=NULL;
  int verbose = 0;

 int displayDataOutputFormat=0;
 int _outputMode=0;

 printf("API inertial sensor - LAAS CNRS 2007\n");

		
  while ((ch = getopt(argc, argv, "vd:o:l:")) != -1)
    {
      switch (ch)
	{
	case 'l':
	  logFile = optarg;	    
	  break;

	case 'd':// d : display data output format  : 	
		// 1 - Calibrated data
		// 2 - Orientation data
		// 3 - Both Calibrated and Orientation data
	    	displayDataOutputFormat = atoi(optarg);		
		break;
	break;

	case 'o':// o : output mode :
		// 1 - Quaternions
		// 2 - Euler angles
		// 3 - Matrix
		
		_outputMode = atoi(optarg);
		break;

	case 'v':
	  verbose = 1;
	  break;
	  	
	case '?':
	default:
	  usage(argv[0]);
	} /* switch */
    }

  argc -= optind;
  argv += optind;
  
  if (argc < 1)
    {
      usage(prog);
      exit(0);
    }

  // open log file
  FILE *fdLog = NULL;
  if(logFile != NULL)
    fdLog = setLogData(logFile);

  // load user settings in inertial sensor
  getUserInputs(argv[0], _outputMode, displayDataOutputFormat);	

  // Open and initialize serial port
#ifdef WIN32
  if (mtcomm.openPort(portNumber) != MTRV_OK)
    {
      printf("Cannot open COM port %d\n", portNumber);
#else
     if (mtcomm.openPort(deviceName) != MTRV_OK)
	{
	  printf("MTI Cannot open COM port %s\n", deviceName);
#endif
	  return MTRV_INPUTCANNOTBEOPENED;
	}	

      if(doMtSettings() == -1)
	return MTRV_UNEXPECTEDMSG;
       
      // output format logged into log file if needed
      if(verbose == 1 || logFile != NULL)
	{
	  memset(msg, 0, 50);
	  sprintf(msg,"MTI Calibrated sensor data - LAAS/CNRS 2006\n");
	  logTrame(fdLog, verbose, msg);
	  memset(msg, 0, 50);
	  sprintf(msg,"ACCX ACCY ACCZ : unity m/s2 \nGYRX GYRY GYRZ : unity rad/s\nMAGNX MAGNY MAGNZ : arbitrary units normalized to earth field strength\nEulerX EulerY EulerZ : unity degree\n");			   
	  logTrame(fdLog, verbose, msg);
	}
	
      while(1)
	{
	  if(mtcomm.readDataMessage(data, datalen) == MTRV_OK)
	    {
	      // get real time clock
	      printHorodatage(msgHorodatage);

	      mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);      		 
	 				
	      if ((outputMode & OUTPUTMODE_CALIB) != 0)
		{
		  // Output Calibrated data
		  mtcomm.getValue(VALUE_CALIB_ACC, fdata, data, BID_MT);	
		      
		  if(verbose == 1 || logFile != NULL)
		    {
		      memset(msg, 0, 50);
		      sprintf(msg,"ACC %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
		    
		      logTrame(fdLog, verbose, msg);
		    }
					
		  mtcomm.getValue(VALUE_CALIB_GYR, fdata, data, BID_MT);		 

		  if(verbose == 1 || logFile != NULL)
		    {
		      memset(msg, 0, 50);
		      sprintf(msg,"GYR %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
		      logTrame(fdLog, verbose, msg);
		    }
					 
		  mtcomm.getValue(VALUE_CALIB_MAG, fdata, data, BID_MT);		

		  if(verbose == 1 || logFile != NULL)
		    {
		      memset(msg, 0, 50);
		      sprintf(msg,"MAGN %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
		      logTrame(fdLog, verbose, msg);
		    }
		}//endif

	      if ((outputMode & OUTPUTMODE_ORIENT) != 0)
		{
		  switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK)
		    {
		    case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
		      // Output: quaternion
		      mtcomm.getValue(VALUE_ORIENT_QUAT, fdata, data, BID_MT);

			if(verbose == 1 || logFile != NULL)
		    	{
		         memset(msg, 0, 50);
		         sprintf(msg,"Quaternion %s %6.3f\t%6.3f\t%6.3f\t%6.3f", msgHorodatage, fdata[0], fdata[1], fdata[2], fdata[3]);
		         logTrame(fdLog, verbose, msg);
		    	}
		      break;
		    case OUTPUTSETTINGS_ORIENTMODE_EULER:
		      // Output: Euler		      
		      mtcomm.getValue(VALUE_ORIENT_EULER, fdata, data, BID_MT);		

		      if(verbose == 1 || logFile != NULL)
			{
			  memset(msg, 0, 50);
			  sprintf(msg,"Euler %s %g %g %g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
			  logTrame(fdLog, verbose, msg);
			}			  

		      break;
		    case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
		      // Output: Cosine Matrix
			mtcomm.getValue(VALUE_ORIENT_MATRIX, fdata, data, BID_MT);
			
			if(verbose == 1 || logFile != NULL)
		    	{
		         memset(msg, 0, 50);
		         sprintf(msg,"Matrix %s %6.3f\t%6.3f\t%6.3f", msgHorodatage, fdata[0], fdata[1], fdata[2]);
		         logTrame(fdLog, verbose, msg);
			
			 memset(msg, 0, 50);
		         sprintf(msg,"Matrix %s %6.3f\t%6.3f\t%6.3f", msgHorodatage, fdata[3], fdata[4], fdata[5]);
		         logTrame(fdLog, verbose, msg);

			 memset(msg, 0, 50);
		         sprintf(msg,"Matrix %s %6.3f\t%6.3f\t%6.3f", msgHorodatage, fdata[6], fdata[7], fdata[8]);
		         logTrame(fdLog, verbose, msg);			
		    	}
		      break;
		    default:
		      ;
		    }// end switch
		} // endif	
	    } //endif check read data
	  else // display error on read buffer
	    {
	      if(verbose == 1 || logFile != NULL)
		{
		  memset(msg, 0, 50);	
		  sprintf(msg, "failed to read message, code (%d)\n", mtcomm.getLastRetVal());
		  logTrame(fdLog, verbose, msg);
		}
	      else
		fprintf(stderr, "MTI failed to read message, code (%d)\n", mtcomm.getLastRetVal());
	    }
	}// end while

      // if data logged then, close fd

      if(fdLog != NULL)
	{	
	  fclose(fdLog);
	}

      // When done, close the serial port
      mtcomm.close();

      return MTRV_OK;
    }


