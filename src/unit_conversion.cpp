//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: unit_conversion.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 30-Nov-2021 15:28:02
//

// Include Files
#include "unit_conversion.h"
#include "rt_nonfinite.h"
#include "unit_conversion_types.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const double Lidar_Detection[256]
//                const struct0_T *LIDAR_DETECTION
//                const struct3_T *UNIT_CONVERSION
//                const struct9_T *DEFINITION
//                coder::array<double, 2U> &Lidar_Detection_out
// Return Type  : void
//
void unit_conversion(const double Lidar_Detection[256],
                     const struct0_T *LIDAR_DETECTION,
                     const struct3_T *UNIT_CONVERSION,
                     const struct9_T *DEFINITION,
                     coder::array<double, 2U> &Lidar_Detection_out)
{
  double b_Lidar_Detection[32];
  // --------------------------------------------------------------------------
  //  Lidar_Detection : [8 X 32]     Input
  // --------------------------------------------------------------------------
  //  1  : Relative Position X
  //  2  : Relative Position Y
  //  3  : Yaw
  //  4  : Length
  //  5  : Width
  //  6  : Height
  //  7 :  Class
  //  8 :  Score
  // --------------------------------------------------------------------------
  //  Lidar_Detection_out : [10 X 32]     Output
  // --------------------------------------------------------------------------
  //  1  : Relative Position X
  //  2  : Relative Position Y
  //  3  : Yaw
  //  4  : Length
  //  5  : Width
  //  6  : Height
  //  7 :  Class
  //  8 :  Score
  //  9  : Preprocessng Relative Position X
  //  10 : Preprocessing Relative Position Y
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  Lidar_Detection_out.set_size(static_cast<int>(LIDAR_DETECTION->STATE_NUMBER),
                               static_cast<int>(LIDAR_DETECTION->TRACK_NUMBER));
  // --------------------------------------------------------------------------
  //  Unit Conversion
  // --------------------------------------------------------------------------
  if (UNIT_CONVERSION->SWITCH == DEFINITION->ON) {
    int i;
    int loop_ub;
    if (1.0 > LIDAR_DETECTION->MEASURE.STATE_NUMBER) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(LIDAR_DETECTION->MEASURE.STATE_NUMBER);
    }
    for (i = 0; i < 32; i++) {
      for (int i1{0}; i1 < loop_ub; i1++) {
        Lidar_Detection_out[i1 + Lidar_Detection_out.size(0) * i] =
            Lidar_Detection[i1 + (i << 3)];
      }
      b_Lidar_Detection[i] =
          Lidar_Detection[(static_cast<int>(
                               LIDAR_DETECTION->MEASURE.REL_POS_X) +
                           (i << 3)) -
                          1] *
          UNIT_CONVERSION->LIDAR_DETECTION.REL_POS_X;
    }
    loop_ub = Lidar_Detection_out.size(1);
    for (i = 0; i < loop_ub; i++) {
      Lidar_Detection_out[(static_cast<int>(
                               LIDAR_DETECTION->TRACKING.REL_POS_X) +
                           Lidar_Detection_out.size(0) * i) -
                          1] = b_Lidar_Detection[i];
    }
    for (i = 0; i < 32; i++) {
      b_Lidar_Detection[i] =
          Lidar_Detection[(static_cast<int>(
                               LIDAR_DETECTION->MEASURE.REL_POS_Y) +
                           (i << 3)) -
                          1] *
          UNIT_CONVERSION->LIDAR_DETECTION.REL_POS_Y;
    }
    loop_ub = Lidar_Detection_out.size(1);
    for (i = 0; i < loop_ub; i++) {
      Lidar_Detection_out[(static_cast<int>(
                               LIDAR_DETECTION->TRACKING.REL_POS_Y) +
                           Lidar_Detection_out.size(0) * i) -
                          1] = b_Lidar_Detection[i];
    }
  } else {
    int loop_ub;
    if (1.0 > LIDAR_DETECTION->MEASURE.STATE_NUMBER) {
      loop_ub = 0;
    } else {
      loop_ub = static_cast<int>(LIDAR_DETECTION->MEASURE.STATE_NUMBER);
    }
    for (int i{0}; i < 32; i++) {
      for (int i1{0}; i1 < loop_ub; i1++) {
        Lidar_Detection_out[i1 + Lidar_Detection_out.size(0) * i] =
            Lidar_Detection[i1 + (i << 3)];
      }
    }
  }
}

//
// File trailer for unit_conversion.cpp
//
// [EOF]
//
