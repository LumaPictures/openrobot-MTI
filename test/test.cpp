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
//	static char *logFile=NULL;
	int verbose = 0;

	OutputMode _outputMode               = MTI_OPMODE_CALIBRATED;
	OutputFormat displayDataOutputFormat = MTI_OPFORMAT_EULER;
	SyncOutMode syncOutSettings          = MTI_SYNCOUTMODE_DISABLED;

	printf("API inertial sensor - LAAS CNRS 2007\n");


	while ((ch = getopt(argc, argv, "vd:o:")) != -1)
	{
		switch (ch)
		{
			case 'o':// d : display data output format  : 	
				// 1 - Quaternions
				// 2 - Euler angles
				// 3 - Matrix
				switch (atoi(optarg)) {
					case 1:
					displayDataOutputFormat = MTI_OPFORMAT_QUAT;		
					break;
					case 2:
					displayDataOutputFormat = MTI_OPFORMAT_EULER;		
					break;
					case 3:
					displayDataOutputFormat = MTI_OPFORMAT_MAT;		
					break;
				}

			case 'd':// o : output mode :
				// 1 - Calibrated data
				// 2 - Orientation data
				// 3 - Both Calibrated and Orientation data
				switch (atoi(optarg)) {
					case 1:
						_outputMode = MTI_OPMODE_CALIBRATED;
						break;
					case 2:
						_outputMode = MTI_OPMODE_ORIENTATION;
						break;
					case 3:
						_outputMode = MTI_OPMODE_BOTH;
						break;
				}


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

	MTI mti(argv[0], _outputMode, displayDataOutputFormat, syncOutSettings);	
	INERTIAL_DATA data;

	while (1) {
		mti.read(&data, true);
	}
}
