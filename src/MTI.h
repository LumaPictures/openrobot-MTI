
#ifndef _MTI_INCLUDE
#define _MTI_INCLUDE


#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include <stdlib.h>
#include <stdio.h>



int initInertialSensor(int mode, char *device);

FILE* setLogData(char *_fileName);

void logTrame(FILE *fd, int modeVerbose, char *message);

void printHorodatage(char *msg);

void getUserInputs(char *device, int mode);

int doMtSettings(void);

int startMTI(int argc, char *argv[]);

float _ACC[4];
float _GYR[4];
float _MAG[4];
float _EULER[4];

#endif

