//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: unit_conversion_types.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 30-Nov-2021 15:28:02
//

#ifndef UNIT_CONVERSION_TYPES_H
#define UNIT_CONVERSION_TYPES_H

// Include Files
#include "rtwtypes.h"

// Type Definitions
struct struct1_T {
  double REL_POS_X;
  double REL_POS_Y;
  double YAW;
  double LENGTH;
  double WIDTH;
  double HEIGHT;
  double CLASS;
  double SCORE;
  double STATE_NUMBER;
  double ID_EXIST;
};

struct struct2_T {
  double REL_POS_X;
  double REL_POS_Y;
  double STATE_NUMBER;
};

struct struct0_T {
  struct1_T MEASURE;
  struct2_T TRACKING;
  double TRACK_NUMBER;
  double STATE_NUMBER;
};

struct struct4_T {
  double YAW;
  double LONG_VEL;
  double LAT_VEL;
  double YAW_RATE;
  double LONG_ACC;
  double LAT_ACC;
  double STEERING_ANGLE;
};

struct struct5_T {
  double REL_POS_X;
  double REL_POS_Y;
  double HEADING_ANGLE;
  double LENGTH;
  double WIDTH;
  double HEIGHT;
  double ID;
  double LABEL;
  double BEHAVIOR_STATE;
  double REL_VEL_X;
  double REL_VEL_Y;
};

struct struct6_T {
  double REL_POS_X;
  double REL_POS_Y;
  double HEADING_ANGLE;
  double LENGTH;
  double WIDTH;
  double HEIGHT;
  double LABEL;
};

struct struct7_T {
  double REL_POS_X;
  double REL_POS_Y;
};

struct struct8_T {
  double REL_POS_X;
  double REL_POS_Y;
  double REL_VEL_X;
  double REL_VEL_Y;
  double HEADING_ANGLE;
};

struct struct3_T {
  double SWITCH;
  struct4_T IN_VEHICLE_SENSOR;
  struct5_T LIDAR;
  struct6_T LIDAR_DETECTION;
  struct7_T FRONT_VISION;
  struct8_T GT_TRACK;
};

struct struct9_T {
  double ON;
  double OFF;
};

struct struct11_T {
  double REL_POS_Y;
  double REL_POS_X;
  double REL_VEL_Y;
  double REL_VEL_X;
  double WIDTH;
  double LENGTH;
  double HEIGHT;
  double HEADING_ANGLE;
  double CLASS;
  double RANGE;
  double ANGLE;
};

struct struct12_T {
  double FUSED_LDT_ID;
};

struct struct13_T {
  double REL_POS_Y;
  double REL_POS_X;
  double REL_VEL_Y;
  double REL_VEL_X;
  double UPDATED_AGE;
  double COASTING_AGE;
  double LIFE_TIME;
  double REL_POS_NORMAL;
  double REL_VEL_TANGENTIAL;
  double REL_VEL_NORMAL;
  double REL_ACC_Y;
  double REL_ACC_X;
};

struct struct14_T {
  double P_LONG;
  double TTC;
  double D_BR;
  double D_W;
  double X_P;
  double I_LONG;
  double DLC;
  double TLC;
  double I_LAT;
  double RSS_X;
  double RSS_Y;
  double FCW_HONDA;
  double FCW_MAZDA;
};

struct struct15_T {
  double ID;
  double RECOGNITION;
};

struct struct16_T {
  double PR_CTRV;
  double MANEUVER;
};

struct struct10_T {
  struct11_T MEASURE;
  struct12_T ASSOCIATION;
  double MEASURE_STATE_NUMBER;
  struct13_T TRACKING;
  struct14_T THREAT;
  struct15_T VEHICLE_RECOGNITION;
  struct16_T VEHICLE_MANEUVER;
  double TRACKING_STATE_NUMBER;
  double THREAT_STATE_NUMBER;
  double RECOGNITION_STATE_NUMBER;
  double MANEUVER_STATE_NUMBER;
  double STATE_NUMBER;
  double TRACK_NUMBER;
};

struct struct19_T {
  double PD;
  double PG;
};

struct struct20_T {
  double DEFAULT_VALUE;
};

struct struct18_T {
  double STATE_NUMBER;
  struct19_T PDAF;
  double VEHICLE_GATING_THRESHOLD;
  struct20_T MAHALANOBIS_DISTANCE;
  double X_ACC_VAR_CV;
  double Y_ACC_VAR_CV;
  double X_ACC_VAR_CA;
  double Y_ACC_VAR_CA;
};

struct struct21_T {
  double STATE_NUMBER;
};

struct struct23_T {
  double INPUT_NUMBER;
  double Y_MIN;
  double Y_MAX;
  double X_MIN;
  double X_MAX;
};

struct struct24_T {
  double INPUT_NUMBER;
};

struct struct25_T {
  double DEFAULT_VALUE;
  double REL_POS_Y;
  double REL_POS_X;
  double REL_VEL_Y;
  double REL_VEL_X;
};

struct struct26_T {
  double Y_MIN;
  double Y_MAX;
  double X_MIN;
  double X_MAX;
  double VY_MIN;
  double VY_MAX;
  double VX_MIN;
  double VX_MAX;
};

struct struct28_T {
  double LDT;
  double FVT;
};

struct struct29_T {
  double DEFAULT_VALUE;
  double REL_POS_Y;
  double REL_POS_X;
  double REL_VEL_X;
  double REL_VEL_Y;
};

struct struct33_T {
  double ROI_X_MIN;
  double ROI_X_MAX;
  double THRESHOLD_Y;
  double THRESHOLD_X;
  double THRESHOLD_VY;
  double THRESHOLD_VX;
};

struct struct34_T {
  double ROI_X_MIN;
  double ROI_X_MAX;
  double ROI_X_SLOPE;
  double THRESHOLD_Y;
  double THRESHOLD_X;
  double THRESHOLD_VY;
  double THRESHOLD_VX;
};

struct struct32_T {
  double ROI_NUMBER;
  struct33_T ROI1;
  struct34_T ROI2;
  struct33_T ROI3;
  struct33_T ROI4;
};

struct struct36_T {
  struct33_T ROI1;
  struct34_T ROI2;
};

struct struct35_T {
  double ROI_NUMBER;
  struct36_T FRONT;
  struct36_T REAR;
};

struct struct31_T {
  struct32_T FVT;
  struct35_T LDT;
};

struct struct30_T {
  struct31_T LDT;
};

struct struct40_T {
  double ROI_X_MIN;
  double ROI_X_MAX;
  double ROI_Y_SLOPE;
  double ROI_X_SLOPE;
  double THRESHOLD_VY;
  double THRESHOLD_VX;
};

struct struct39_T {
  double ROI_NUMBER;
  struct33_T ROI1;
  struct40_T ROI2;
  struct33_T ROI3;
};

struct struct38_T {
  struct39_T FVT;
  struct35_T LDT;
};

struct struct37_T {
  struct38_T LDT;
};

struct struct27_T {
  double SENSOR_NUMBER;
  struct28_T MAP;
  struct24_T GATING;
  struct29_T RESIDUAL;
  struct30_T VEHICLE_GATING;
  struct37_T PEDESTRIAN_GATING;
};

struct struct22_T {
  double X_ACC_VAR_CV;
  double Y_ACC_VAR_CV;
  double H_CV[16];
  double R_CV[36];
  double DELETE_TRACK_COUNT;
  double H_CA[36];
  double R_CA[36];
};

struct struct17_T {
  struct18_T FUSION_TRACK;
  struct21_T MEASURE;
  double LIDAR_ASSOCIATION_MAP[32];
  struct22_T LIDAR;
  struct23_T LIDAR_GATING;
  double ASSOCIATION_MAP[32];
  struct24_T GATING;
  struct25_T RESIDUAL;
  struct26_T VEHICLE_GATING;
  double THRESHOLD_MAX_LIFE_TIME;
  double DELETE_TRACK_COUNT;
};

#endif
//
// File trailer for unit_conversion_types.h
//
// [EOF]
//
