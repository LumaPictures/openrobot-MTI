#include "../MTI/MTI.h"
#include "../MTI/MTComm.h"

#include <stdlib.h>

/*! 
  @brief : Utilisation du programme principal
*/
static void usage(const char *progname)
{
  fprintf(stderr, "usage: %s [-v] serial_device [-o] mode [-d] mode display\n", progname);
  exit(1);
}

int main(int argc, char *argv[])
{
	int  ch;
	char *prog = argv[0];
	static char *logFile=NULL;
	int verbose = 0;

	int displayDataOutputFormat=0;
	int _outputMode=0;

	printf("API inertial sensor - LAAS CNRS 2007\n");


	while ((ch = getopt(argc, argv, "vd:o:")) != -1)
	{
		switch (ch)
		{
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

	MTI mti(argv[0], _outputMode, displayDataOutputFormat);	
	INERTIAL_DATA data;

	while (1) {
		mti.read(&data, true);
	}
}
