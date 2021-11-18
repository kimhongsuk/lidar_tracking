//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fusion_object.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Nov-2021 11:27:25
//

#ifndef FUSION_OBJECT_H
#define FUSION_OBJECT_H

// Include Files
#include "rtwtypes.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void fusion_object(const double Lidar_Detection[320],
                          const struct10_T *FUSION_TRACK,
                          const struct0_T *LIDAR_DETECTION,
                          coder::array<double, 2U> &Fusion_Object);

#endif
//
// File trailer for fusion_object.h
//
// [EOF]
//
