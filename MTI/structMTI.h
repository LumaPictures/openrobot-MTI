
// Contient toutes les données à poster utilisées
// par GENOM

#ifndef STRUCT_MTI_H
#define STRUCT_MTI_H

typedef struct INERTIAL_CONFIG
{
  
  char device[24]; // dev/ttyS0 par exemple

// 1 - Calibrated data
// 2 - Orientation data\n");
// 3 - Both Calibrated and Orientation data\n
  int mode;

// 1 - Quaternions
// 2 - Euler angles
// 3 - Matrix
  int outputMode; 
} INERTIAL_CONFIG;

  
typedef struct INERTIAL_DATA
{
  float ACC[4];
  float GYR[4];
  float MAG[4];
  float EULER[4];
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
