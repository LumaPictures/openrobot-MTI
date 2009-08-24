
// Contient toutes les donn�es � poster utilis�es
// par GENOM

#ifndef STRUCT_MTI_H
#define STRUCT_MTI_H

typedef enum OutputMode {
	MTI_OPMODE_CALIBRATED  = 0x02, 
	MTI_OPMODE_ORIENTATION = 0x04, 
	MTI_OPMODE_BOTH        = 0x06
} OutputMode;

typedef enum OutputFormat {
	MTI_OPFORMAT_QUAT  = 0x00,
	MTI_OPFORMAT_EULER = 0x04,
	MTI_OPFORMAT_MAT   = 0x08
} OutputFormat;

typedef enum SyncOutMode {
	MTI_SYNCOUTMODE_DISABLED = 0,
	MTI_SYNCOUTMODE_TOGGLE   = 1,
	MTI_SYNCOUTMODE_PULSE    = 2
} SyncOutMode;

typedef enum SyncOutPulsePolarity {
	MTI_SYNCOUTPULSE_NEG = 0x00,
	MTI_SYNCOUTPULSE_POS = 0x16
} SyncOutPulsePolarity;

typedef struct INERTIAL_DEVICE
{
  char device[24]; // dev/ttyS0 par exemple
} INERTIAL_DEVICE;


typedef struct INERTIAL_CONFIG
{
  OutputMode   outputMode;
  OutputFormat outputFormat; 

  SyncOutMode          syncOutMode;
  SyncOutPulsePolarity syncOutPulsePolarity;

  // number of acquisitions per syncOut marks
  int syncOutSkipFactor;
  
  // number of clock ticks @ 33.9ns
  int syncOutOffset;
  
  // number of clock ticks @ 33.9ns
  int syncOutPulseWidth;
  
} INERTIAL_CONFIG;


typedef struct INERTIAL_DATA
{
  float ACC[3];
  float GYR[3];
  float MAG[3];
  float EULER[3];
  int   COUNT;
} INERTIAL_DATA;

/* AVERAGING_PARAMETERS is used to average the values on the roll and pitch
   angles */
typedef struct AVERAGING_PARAMETERS
{
  int	nbSamples;
} AVERAGING_PARAMETERS;

/* MTI_BIASES contains the estimated bias values on the roll and pitch angles
   (in degrees) */
typedef struct MTI_BIASES
{
  double	pitchBias;
  double	rollBias;	/* In degrees */
} MTI_BIASES;


#endif
