
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

#include "MTComm.h"
#include <stdio.h>
#include <errno.h>


#include <time.h>
#include <unistd.h>
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
  fprintf(stderr, "usage: %s [-v] serial_device [-l] logFile\n", progname);
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
  @brief Procédure qui initialise une structure de données de temps

  @param struct timeval *timev : structure de temps à modifier
*/
void getHorodatage(struct timeval *timev)
{
  
  // obsolete en 2038 !! je serai encore la ??
  if(gettimeofday(timev, NULL) !=0)
    {
      perror("gettimeofday");

      exit(EXIT_FAILURE);
    }
}

/*! 
  @brief Affiche le contenu d'une structure de données sous forme ASCII

  @param struct timeval timev : affiche le contenu
*/
void printHorodatage(char *msg)
{
  struct tm* ptm;
  char time_string[40];
  long milliseconds;
  struct timeval timev;
     
  // System time
  getHorodatage(&timev);

  ptm = localtime (&timev.tv_sec);

  /* Compute milliseconds from microseconds. */
  milliseconds = timev.tv_usec / 1000;
  
  /* Format the date and time, down to a single second. */
  strftime (time_string, sizeof (time_string), "%Y-%m-%d %H:%M:%S", ptm);
  
  /* Print the formatted time, in seconds, followed by a decimal point
     and the milliseconds. */
  sprintf (msg,"%s.%03ld", time_string, milliseconds);
}



/*!
  @brief : user setting
*/

void getUserInputs(char *device)
{
#ifdef WIN32
  printf("Enter COM port: ");
  scanf("%d", &portNumber);
#else
  strcpy(deviceName, device);
#endif
	
  outputMode = 3;

  // Update outputMode to match data specs of SetOutputMode
  outputMode <<= 1;

  outputSettings = OUTPUTSETTINGS_ORIENTMODE_EULER;

  // outputSettings = OUTPUTSETTINGS_ORIENTMODE_QUATERNION;
  // outputSettings =  OUTPUTSETTINGS_ORIENTMODE_MATRIX;

  outputSettings |= OUTPUTSETTINGS_TIMESTAMP_SAMPLECNT;
}



/*!
  @brief : recording user settings into inertial sensor befor measurement
*/
bool doMtSettings(void) 
{
  unsigned long tmpOutputMode, tmpOutputSettings;
  unsigned short tmpDataLength;

  // Put MTi/MTx in Config State
  if(mtcomm.writeMessage(MID_GOTOCONFIG) != MTRV_OK)
    {
      printf("No device connected\n");
      return false;
    }

  // Get current settings and check if Xbus Master is connected
  if (mtcomm.getDeviceMode(&numDevices) != MTRV_OK)
    {
      if (numDevices == 1)
	printf("MTi / MTx has not been detected\nCould not get device mode\n");
      else
	printf("Not just MTi / MTx connected to Xbus\nCould not get all device modes\n");
      return false;
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
	  return false;
	}
    }

  // Put MTi/MTx in Measurement State
  mtcomm.writeMessage(MID_GOTOMEASUREMENT);

  return true;
}




/*!
  @brief : main
*/
int main(int argc, char *argv[]) 
{
  unsigned char data[MAXMSGLEN];
  short datalen;
  float fdata[18] = {0};
  unsigned short samplecounter;

  // convert float to string to log
  char msg[50];
  char msgHorodatage[100];

  // Skip factor for writing data to screen, make screen seem a bit smoother
  short screenSkipFactor = 10;
  short screenSkipFactorCnt = screenSkipFactor;

  int  ch;
  char *prog = argv[0];
  static char *logFile=NULL;
  int verbose = 0;
		
  while ((ch = getopt(argc, argv, "vl:")) != -1)
    {
      switch (ch)
	{
	case 'l':
	  logFile = optarg;	    
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
    }

  // open log file
   FILE *fdLog = NULL;
   if(logFile != NULL)
     fdLog = setLogData(logFile);

  // load user settings in inertial sensor
  getUserInputs(argv[0]);	

  // Open and initialize serial port
#ifdef WIN32
  if (mtcomm.openPort(portNumber) != MTRV_OK)
    {
      printf("Cannot open COM port %d\n", portNumber);
#else
      if (mtcomm.openPort(deviceName) != MTRV_OK)
	{
	  printf("Cannot open COM port %s\n", deviceName);
#endif
	  return MTRV_INPUTCANNOTBEOPENED;
	}	

      if(doMtSettings() == false)
	return MTRV_UNEXPECTEDMSG;

      printf("Calibrated sensor data\n");
	
      while(mtcomm.readDataMessage(data, datalen) == MTRV_OK )
	{
	  mtcomm.getValue(VALUE_SAMPLECNT, samplecounter, data, BID_MASTER);
		
	  if (screenSkipFactorCnt++ == screenSkipFactor)
	    {
	      screenSkipFactorCnt = 0;

	      for (int i = 0; i < numDevices; i++)
		{				
		  if ((outputMode & OUTPUTMODE_CALIB) != 0)
		    {
		      // Output Calibrated data
		      mtcomm.getValue(VALUE_CALIB_ACC, fdata, data, BID_MT + i);

		      printHorodatage(msgHorodatage);

		   /*   printf("ACCX:%6.2f\t  ACCY:%6.2f\t ACCX:%6.2f UNITY:(m/s^2)\n", fdata[0], 
			     fdata[1], 
			     fdata[2]); */
		      
		      if(verbose == 1 || logFile != NULL)
			{
			  memset(msg, 0, 50);
			  sprintf(msg,"%s ACCX:%g ACCY:%g ACCZ:%g", msgHorodatage, fdata[0], fdata[1], fdata[2]);			   
			  logTrame(fdLog, verbose, msg);
			}
			
		      printHorodatage(msgHorodatage);
		      mtcomm.getValue(VALUE_CALIB_GYR, fdata, data, BID_MT + i);
		      /* printf("GYRX:%6.2f\t GYRY:%6.2f\t GYRZ:%6.2f  UNITY:(rad/s)\n", fdata[0], fdata[1], fdata[2]); */

		      if(verbose == 1 || logFile != NULL)
			{
			  memset(msg, 0, 50);
			  sprintf(msg,"%s GYRX:%g GYRY:%g GYRZ:%g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
			  logTrame(fdLog, verbose, msg);
			}
			

		      printHorodatage(msgHorodatage);
		      mtcomm.getValue(VALUE_CALIB_MAG, fdata, data, BID_MT + i);
		      /* printf("MAGX :%6.2f\t MAGY:%6.2f\t MAGZ:%6.2f  UNITY:(a.u.)\n", fdata[0], fdata[1], fdata[2]);	 */

		      if(verbose == 1 || logFile != NULL)
			{
			  memset(msg, 0, 50);
			  sprintf(msg,"%s MAGNX:%g MAGNY:%g MAGNZ:%g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
			  logTrame(fdLog, verbose, msg);
			}
		    }

		  if ((outputMode & OUTPUTMODE_ORIENT) != 0)
		    {
		      switch(outputSettings & OUTPUTSETTINGS_ORIENTMODE_MASK)
			{
			case OUTPUTSETTINGS_ORIENTMODE_QUATERNION:
			  // Output: quaternion
			  mtcomm.getValue(VALUE_ORIENT_QUAT, fdata, data, BID_MT + i);
			  printf("%6.3f\t%6.3f\t%6.3f\t%6.3f\n",
				 fdata[0],
				 fdata[1], 
				 fdata[2], 
				 fdata[3]); 
			  break;
			case OUTPUTSETTINGS_ORIENTMODE_EULER:
			  // Output: Euler
			  printHorodatage(msgHorodatage);
			  mtcomm.getValue(VALUE_ORIENT_EULER, fdata, data, BID_MT + i);		

			   if(verbose == 1 || logFile != NULL)
			     {
			       memset(msg, 0, 50);
			       sprintf(msg,"%s EulerX:%g EulerY:%g EulerZ:%g", msgHorodatage, fdata[0], fdata[1], fdata[2]);
			       logTrame(fdLog, verbose, msg);
			     }			  

			  break;
			case OUTPUTSETTINGS_ORIENTMODE_MATRIX:
			  // Output: Cosine Matrix
			  mtcomm.getValue(VALUE_ORIENT_MATRIX, fdata, data, BID_MT + i);
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
			}
		    }
		}
	    }
	}
      // if data logged then, close fd
      if(fdLog != NULL)
	{	
	  fclose(fdLog);
	}

      // When done, close the serial port
      mtcomm.close();

      return MTRV_OK;
    }
