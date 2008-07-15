
#ifndef _MTI_INCLUDE
#define _MTI_INCLUDE

#include "structMTI.h"
#include "MTComm.h"

class MTI {
	private:
		const char * device;
		int mode;
		int outputDisplay;
		CMTComm mtcomm;
		bool connected;

		bool _set_mode(int mode_);
		bool _set_outputDisplay(int outputDisplay_);
		bool _configure_device();

	public:
		MTI(const char *dev_, int mode_, int outputDisplay_);
		~MTI();

		bool connect();
		bool connect(const char * dev);
		bool disconnect();

		bool is_connected() { return connected; };

		bool set_mode(int mode_);
		bool set_outputDisplay(int outputDisplay_);

		bool read(INERTIAL_DATA * output, bool verbose=false);
};

#endif

