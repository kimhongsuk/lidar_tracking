//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: correction.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Nov-2021 11:27:25
//

#ifndef CORRECTION_H
#define CORRECTION_H

// Include Files
#include "rtwtypes.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void
correction(const double Fusion_Object[352],
           const double Association_Map_k_1[32],
           const double Fusion_Track_Predicted[768],
           const double P_Fusion_Track_Predicted[1152],
           const struct10_T *FUSION_TRACK, const struct17_T *TRACKING,
           const struct27_T *ASSOCIATION, double Fusion_Track_Updated[768],
           coder::array<double, 3U> &P_Fusion_Track_Updated,
           coder::array<double, 2U> &Association_Map_Updated,
           coder::array<double, 3U> &S_CA, coder::array<double, 2U> &Md_FV);

#endif
//
// File trailer for correction.h
//
// [EOF]
//
