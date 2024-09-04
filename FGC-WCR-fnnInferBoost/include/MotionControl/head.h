#pragma once
#ifndef head_H
#define head_H

// #define VMCCONTROL
#define FNNCONTROL
#define ForceLPF  0.3
/* Admittance control */
// #define StepHeight_F  17.0*0.001  //swingUp
// #define StepHeight_H  16.0*0.001  //swingUp
// #define Press  16.0*0.001       //attach press 
// #define CompDisA1 -4*0.001     // Compensation of Distance in AttitudeCorrection() with Amble gait
// #define CompDisA2 26*0.001  
// #define CompDisA3 30*0.001  
// #define CompDisALL 16*0.001     // Compensation of Distance  AttitudeCorrection() All stace phase
/* Position control */
#define StepHeight_F  16.0/1000   //swingUp
#define StepHeight_H  14.0/1000   //swingUp
#define Press  0/1000.0  //19.0/1000        //attach press 7
#define CompDisA1 -2.0/1000.0   // Compensation of Distance in AttitudeCorrection() with Amble gait
#define CompDisA2 12.0/1000.0 
#define CompDisA3 14.0/1000.0   
#define CompDisALL 9.0 /1000.0     // Compensation of Distance  AttitudeCorrection() All stace phase

#define THREAD1_ENABLE 1
#define THREAD2_ENABLE 1
//  1:  Motor angle
//  2:  Foot end position
#define INIMODE 2
#define MORTOR_ANGLE_AMP 40*3.14/180.0
#define loopRateCommandUpdate 100.0   //hz
#define loopRateStateUpdateSend 20.0   //hz
#define loopRateImpCtller 100.0   //hz
#define loopRateDataSave 100 //hz
#define loopRateSVRead   20.0//hz
#define VELX 6.0/1000    // mm  step length = VELX * timeForStancePhase        
#define TimePeriod 0.05
#define TimeForGaitPeriod 8.0
#define PI 3.1415926
#define THREHOLDLF 800
#define THREHOLDRF 800
#define THREHOLDLH 800
#define THREHOLDRH 800
#define ATTACHDIS_MAX 30.0/1000
#define ATTACH_TIMES  20
#define PrePsotiveFactor 0.04




#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <time.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>
#include <map>
#include <string>
#include <string.h>
#include "i2c.h"
#include "dynamixel.h"
#include "api.h"
#include <stdio.h>
#include <wiringPi.h> 
#include<fstream>
#include<sstream>
#include<bitset>
#ifdef  VMCCONTROL
  #include <qpOASES.hpp>
#endif
using namespace Eigen;
using namespace std;
#endif
