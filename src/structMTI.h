
// Contient toutes les données à poster utilisées
// par GENOM

#ifndef STRUCT_MTI_H
#define STRUCT_MTI_H

typedef struct INERTIAL_INIT
{
  
  char device[24]; // dev/ttyS0 par exemple
  int mode; // Euler, Matrix, Quaternion
  
  float ACC[4];
  float GYR[4];
  float MAG[4];
  float EULER[4];

} INERTIAL_INIT;

#endif
