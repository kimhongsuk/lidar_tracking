//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fusion_track_management.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Nov-2021 11:27:25
//

#ifndef FUSION_TRACK_MANAGEMENT_H
#define FUSION_TRACK_MANAGEMENT_H

// Include Files
#include "rtwtypes.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void fusion_track_management(
    const double Fusion_Object[352], const double Fusion_Track_Updated[768],
    const double P_Fusion_Track_Updated[1152], float Association_Map_Updated,
    float Association_Map_k_1, const struct17_T *TRACKING,
    const struct9_T *DEFINITION, double LIDAR_TRACK_SWITCH,
    const struct10_T *FUSION_TRACK, const struct27_T *ASSOCIATION,
    double FRONT_VISION_TRACK_SWITCH,
    coder::array<double, 2U> &Fusion_Track_out, float *Association_Map_out,
    coder::array<double, 3U> &P_Fusion_Track_out);

#endif
//
// File trailer for fusion_track_management.h
//
// [EOF]
//
