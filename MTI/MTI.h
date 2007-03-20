


#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "MTComm.h"


#ifndef MTI_H
#define MTI_H

void usage(const char *progname);

FILE* setLogData(char *_fileName);

void logTrame(FILE *fd, int modeVerbose, char *message);

void printHorodatage(char *msg);

void getUserInputs(char *device, int *portNumber, char *deviceName, int *outputMode,  int *outputSettings);

bool doMtSettings(CMTComm mtcomm, int outputMode, int *outputSettings, unsigned short *numDevices);

int initInertialSensor(int mode, char *device);
#endif
