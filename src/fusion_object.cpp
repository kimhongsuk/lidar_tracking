//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fusion_object.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 30-Nov-2021 15:28:02
//

// Include Files
#include "fusion_object.h"
#include "rt_nonfinite.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include "rt_defines.h"
#include <cmath>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : const double Lidar_Detection[320]
//                const struct10_T *FUSION_TRACK
//                const struct0_T *LIDAR_DETECTION
//                coder::array<double, 2U> &Fusion_Object
// Return Type  : void
//
void fusion_object(const double Lidar_Detection[320],
                   const struct10_T *FUSION_TRACK,
                   const struct0_T *LIDAR_DETECTION,
                   coder::array<double, 2U> &Fusion_Object)
{
  int i;
  // --------------------------------------------------------------------------
  //  Lidar_Detection : [10 X 32]     Input
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
  //  10  : Preprocessing Relative Position Y
  // --------------------------------------------------------------------------
  //  Fusion Object : [11 X 32]     Output
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  //  5  : Width
  //  6  : Length
  //  7  : Height
  //  8  : Heading angle
  //  9  : Range
  //  10 : Angle
  //  11 : Fused LDT ID
  // --------------------------------------------------------------------------
  //  Initialization
  // --------------------------------------------------------------------------
  i = static_cast<int>(FUSION_TRACK->TRACK_NUMBER);
  Fusion_Object.set_size(static_cast<int>(FUSION_TRACK->MEASURE_STATE_NUMBER),
                         i);
  // --------------------------------------------------------------------------
  //  Creation of Fusion Object
  // --------------------------------------------------------------------------
  for (int index_FST{0}; index_FST < i; index_FST++) {
    double a_tmp;
    //
    //      elseif LIDAR_TRACK_SWITCH == DEFINITION.ON &&
    //      FRONT_VISION_TRACK_SWITCH == DEFINITION.OFF
    a_tmp = Lidar_Detection[(static_cast<int>(LIDAR_DETECTION->MEASURE.LENGTH) +
                             10 * index_FST) -
                            1];
    if (a_tmp != 0.0) {
      //  id -> find index
      //              [tmp_LIDAR_index] = Find_Sensor_Index(tmp_LIDAR_ID,
      //              Lidar_Detection, LIDAR_TRACK);
      //              Fusion_Object(FUSION_TRACK.MEASURE.REL_POS_Y, index_FST)
      //              =
      //              Lidar_Track(LIDAR_TRACK.PREPROCESSING.REAR_REL_POS_Y,tmp_LIDAR_index);
      //              Fusion_Object(FUSION_TRACK.MEASURE.REL_POS_X, index_FST)
      //              = Lidar_Track(LIDAR_TRACK.PREPROCESSING.REAR_REL_POS_X,
      //              tmp_LIDAR_index);
      //                  Fusion_Object(FUSION_TRACK.MEASURE.REL_POS_Y,
      //                  index_FST)           =
      //                  Lidar_Track(LIDAR_TRACK.PREPROCESSING.REAR_REL_POS_Y,tmp_LIDAR_index);
      //                  Fusion_Object(FUSION_TRACK.MEASURE.REL_POS_X,
      //                  index_FST)           =
      //                  Lidar_Track(LIDAR_TRACK.PREPROCESSING.REAR_REL_POS_X,
      //                  tmp_LIDAR_index);
      if (Lidar_Detection[(static_cast<int>(LIDAR_DETECTION->MEASURE.CLASS) +
                           10 * index_FST) -
                          1] == 1.0) {
        double b_a_tmp;
        b_a_tmp = Lidar_Detection[(static_cast<int>(
                                       LIDAR_DETECTION->TRACKING.REL_POS_Y) +
                                   10 * index_FST) -
                                  1];
        if ((b_a_tmp < 7.0) && (b_a_tmp > -7.0)) {
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.REL_POS_Y) +
                         Fusion_Object.size(0) * index_FST) -
                        1] = b_a_tmp;
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.REL_POS_X) +
                         Fusion_Object.size(0) * index_FST) -
                        1] =
              Lidar_Detection[(static_cast<int>(
                                   LIDAR_DETECTION->TRACKING.REL_POS_X) +
                               10 * index_FST) -
                              1];
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.WIDTH) +
                         Fusion_Object.size(0) * index_FST) -
                        1] =
              Lidar_Detection[(static_cast<int>(
                                   LIDAR_DETECTION->MEASURE.WIDTH) +
                               10 * index_FST) -
                              1];
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.LENGTH) +
                         Fusion_Object.size(0) * index_FST) -
                        1] = a_tmp;
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.HEIGHT) +
                         Fusion_Object.size(0) * index_FST) -
                        1] =
              Lidar_Detection[(static_cast<int>(
                                   LIDAR_DETECTION->MEASURE.HEIGHT) +
                               10 * index_FST) -
                              1];
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.HEADING_ANGLE) +
                         Fusion_Object.size(0) * index_FST) -
                        1] =
              Lidar_Detection[(static_cast<int>(LIDAR_DETECTION->MEASURE.YAW) +
                               10 * index_FST) -
                              1];
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.CLASS) +
                         Fusion_Object.size(0) * index_FST) -
                        1] = 1.0;
          a_tmp =
              Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.REL_POS_X) +
                             Fusion_Object.size(0) * index_FST) -
                            1];
          b_a_tmp =
              Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.REL_POS_Y) +
                             Fusion_Object.size(0) * index_FST) -
                            1];
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.RANGE) +
                         Fusion_Object.size(0) * index_FST) -
                        1] = std::sqrt(a_tmp * a_tmp + b_a_tmp * b_a_tmp);
          Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.ANGLE) +
                         Fusion_Object.size(0) * index_FST) -
                        1] =
              rt_atan2d_snf(
                  Fusion_Object[(static_cast<int>(
                                     FUSION_TRACK->MEASURE.REL_POS_Y) +
                                 Fusion_Object.size(0) * index_FST) -
                                1],
                  Fusion_Object[(static_cast<int>(
                                     FUSION_TRACK->MEASURE.REL_POS_X) +
                                 Fusion_Object.size(0) * index_FST) -
                                1]);
          Fusion_Object[(static_cast<int>(
                             FUSION_TRACK->ASSOCIATION.FUSED_LDT_ID) +
                         Fusion_Object.size(0) * index_FST) -
                        1] = static_cast<double>(index_FST) + 1.0;
        }
      }
      //              Fusion_Object(FUSION_TRACK.MOTION_ATTRIBUTE.ABS_VEL,
      //              index_FST)         =
      //              Lidar_Detection(LIDAR_TRACK.MOTION_ATTRIBUTE.ABS_VEL,
      //              tmp_LIDAR_index);
      //              Fusion_Object(FUSION_TRACK.MOTION_ATTRIBUTE.MOTION,
      //              index_FST)         =
      //              Lidar_Detection(LIDAR_TRACK.MOTION_ATTRIBUTE.MOTION,
      //              tmp_LIDAR_index);
    }
  }
}

//
// File trailer for fusion_object.cpp
//
// [EOF]
//
