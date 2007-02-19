

#ifndef WIN32
#include <sys/ioctl.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

#include "../MTI/MTI.h"
#include "../MTI/MTComm.h"



int main(int argc, char *argv[])
{
 return startMTI(argc, argv);

}
