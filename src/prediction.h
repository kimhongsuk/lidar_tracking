//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: prediction.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Nov-2021 11:27:25
//

#ifndef PREDICTION_H
#define PREDICTION_H

// Include Files
#include "rtwtypes.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void prediction(const double Association_Map_k_1[32],
                       const double Fusion_Track_k_1[768],
                       const double P_Fusion_Track_k_1[1152],
                       double SAMPLE_TIME, const struct10_T *FUSION_TRACK,
                       const struct17_T *TRACKING,
                       coder::array<double, 2U> &Fusion_Track_Predicted,
                       coder::array<double, 3U> &P_Fusion_Track_Predicted);

#endif
//
// File trailer for prediction.h
//
// [EOF]
//
