
#ifndef _MTI_INCLUDE
#define _MTI_INCLUDE

#include "structMTI.h"
#include "MTComm.h"

class MTI {
	private:
		const char * device;
		int mode;
		int outputDisplay;
		int syncOutMode;
		int syncOutPulsePolarity;
		int syncOutSkipFactor;
		int syncOutOffset;
		int syncOutPulseWidth;
		int outputSkipFactor;
		CMTComm mtcomm;
		bool connected;

		bool _set_mode(OutputMode mode_);
		bool _set_outputDisplay(OutputFormat outputDisplay_);
		bool _set_syncOut(SyncOutMode syncOutMode_                   = MTI_SYNCOUTMODE_PULSE,
				  SyncOutPulsePolarity syncOutPulsePolarity_ = MTI_SYNCOUTPULSE_POS,
				  int syncOutSkipFactor_    = 0, // output all pulses
				  int syncOutOffset_        = 0, // pulse at acq. time
				  int syncOutPulseWidth_    = 2040); // 69.156us pulse
		bool _set_outputSkipFactor(int factor);
		bool _configure_device();
		bool _reset();

	public:
		MTI(const char *dev_, 
		    OutputMode mode_, 
		    OutputFormat outputDisplay_, 
		    SyncOutMode syncOutMode_ = MTI_SYNCOUTMODE_DISABLED);
		~MTI();

		bool connect();
		bool connect(const char * dev);
		bool disconnect();

		bool is_connected() { return connected; };

		bool set_mode(OutputMode mode_);
		
		bool set_outputDisplay(OutputFormat outputDisplay_);
		
		bool set_syncOut(SyncOutMode          syncOutMode_, 
				 SyncOutPulsePolarity syncOutPulsePolarity_, 
				 int syncOutSkipFactor_,
				 int syncOutOffset_,
				 int syncOutPulseWidth_);
		bool set_outputSkipFactor(int factor);
				 
		bool read(INERTIAL_DATA * output, bool verbose=false);
		bool reset();
};

#endif

