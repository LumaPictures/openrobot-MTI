
#ifndef _MTI_INCLUDE
#define _MTI_INCLUDE


#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "structMTI.h"

int initInertialSensor();
int readInertialSensor(INERTIAL_DATA* data);
void closeInertialSensor();

FILE* setLogData(char *_fileName);

void logTrame(FILE *fd, int modeVerbose, char *message);

void printHorodatage(char *msg);

void getUserInputs( char *device, int mode, int outputDisplay);

int doMtSettings(void);

int startMTI(int argc, char *argv[]);

#endif

