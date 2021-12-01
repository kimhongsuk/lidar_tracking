//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fusion_track_management.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 30-Nov-2021 15:28:02
//

// Include Files
#include "fusion_track_management.h"
#include "rt_nonfinite.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>

// Function Definitions
//
// Arguments    : const double Fusion_Object[352]
//                const double Fusion_Track_Updated[768]
//                const double P_Fusion_Track_Updated[1152]
//                const double Association_Map_Updated[32]
//                const double Association_Map_k_1[32]
//                const struct17_T *TRACKING
//                const struct9_T *DEFINITION
//                double LIDAR_TRACK_SWITCH
//                const struct10_T *FUSION_TRACK
//                const struct27_T *ASSOCIATION
//                double FRONT_VISION_TRACK_SWITCH
//                coder::array<double, 2U> &Fusion_Track_out
//                double Association_Map_out[32]
//                coder::array<double, 3U> &P_Fusion_Track_out
// Return Type  : void
//
void fusion_track_management(
    const double Fusion_Object[352], const double Fusion_Track_Updated[768],
    const double P_Fusion_Track_Updated[1152],
    const double Association_Map_Updated[32],
    const double Association_Map_k_1[32], const struct17_T *TRACKING,
    const struct9_T *DEFINITION, double LIDAR_TRACK_SWITCH,
    const struct10_T *FUSION_TRACK, const struct27_T *,
    double FRONT_VISION_TRACK_SWITCH,
    coder::array<double, 2U> &Fusion_Track_out, double Association_Map_out[32],
    coder::array<double, 3U> &P_Fusion_Track_out)
{
  coder::array<signed char, 2U> b_I;
  double b_P_Fusion_Track_Updated[36];
  double b_Fusion_Track_Updated[24];
  double d;
  double t;
  int Fusion_Object_Assigned_Flag;
  int P_Fusion_Track_Updated_tmp;
  int b_P_Fusion_Track_out;
  int i;
  int i1;
  int i2;
  int id_FO;
  int id_FST;
  int loop_ub;
  bool exitg1;
  // --------------------------------------------------------------------------
  //  Fusion_Track_Updated : [24 X 32]     Input
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  //  5  : Width
  //  6  : Length
  //  7  : Height
  //  8  : Heading angle
  //  9  : Class
  //  10  : Range
  //  11 : Angle
  //  12 : Fused Lidar Track ID
  //  13 : (Tracking) Relative position Y
  //  14 : (Tracking) Relative position X
  //  15 : (Tracking) Relative velocity Y
  //  16 : (Tracking) Relative velocity X
  //  17 : Updated Age
  //  18 : Coasting Age
  //  19 : Life time
  //  20 : Relative normal position
  //  21 : Relative tangential velocity with respect to lane
  //  22 : Relative normal velocity with respect to lane
  //  23 : (Tracking) Relative acceleration Y
  //  24 : (Tracking) Relative acceleration X
  // --------------------------------------------------------------------------
  //  P_Fusion_Track_Updated : [4 X 4 X 32]  Input
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  // --------------------------------------------------------------------------
  //  Association_Map_Updated : [32 X 1]     Input
  // --------------------------------------------------------------------------
  //  1  : Lidar Track
  // --------------------------------------------------------------------------
  //  Fusion_Track_out : [24 X 32]     Output
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  //  5  : Width
  //  6  : Length
  //  7  : Height
  //  8  : Heading angle
  //  9  : Class
  //  10  : Range
  //  11 : Angle
  //  12 : Fused Lidar Track ID
  //  13 : (Tracking) Relative position Y
  //  14 : (Tracking) Relative position X
  //  15 : (Tracking) Relative velocity Y
  //  16 : (Tracking) Relative velocity X
  //  17 : Updated Age
  //  18 : Coasting Age
  //  19 : Life time
  //  20 : Relative normal position
  //  21 : Relative tangential velocity with respect to lane
  //  22 : Relative normal velocity with respect to lane
  //  23 : (Tracking) Relative acceleration Y
  //  24 : (Tracking) Relative acceleration X
  // --------------------------------------------------------------------------
  //  Association_Map_out : [32 X 1]     Output
  // --------------------------------------------------------------------------
  //  1  : Lidar Track
  // --------------------------------------------------------------------------
  //  P_Fusion_Track_out : [4 X 4 X 32]     Output
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  i = static_cast<int>(FUSION_TRACK->TRACK_NUMBER);
  Fusion_Track_out.set_size(
      static_cast<int>(FUSION_TRACK->TRACKING_STATE_NUMBER), i);
  P_Fusion_Track_out.set_size(
      static_cast<int>(TRACKING->FUSION_TRACK.STATE_NUMBER),
      static_cast<int>(TRACKING->FUSION_TRACK.STATE_NUMBER), i);
  // Association_Map_out = coder.nullcopy(zeros(FUSION_TRACK.TRACK_NUMBER,
  // ASSOCIATION.SENSOR_NUMBER));
  std::copy(&Association_Map_Updated[0], &Association_Map_Updated[32],
            &Association_Map_out[0]);
  Fusion_Object_Assigned_Flag = 0;
  // Fusion_Object_Exist_Flag = 0;
  // CHECK_THRESHOLD = 1.5;
  // --------------------------------------------------------------------------
  //  Track management - Maintenance
  // --------------------------------------------------------------------------
  for (id_FO = 0; id_FO < i; id_FO++) {
    //      if sum(Association_Map(id_FO,:)) ~= 0 % 검출 여부
    if (Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.LENGTH) +
                       11 * id_FO) -
                      1] != 0.0) {
      t = 1.0;
      id_FST = 0;
      exitg1 = false;
      while ((!exitg1) && (id_FST <= i - 1)) {
        t = static_cast<double>(id_FST) + 1.0;
        if ((Association_Map_Updated[id_FST] != 0.0) &&
            (Association_Map_Updated[id_FST] ==
             Fusion_Object[(static_cast<int>(
                                FUSION_TRACK->ASSOCIATION.FUSED_LDT_ID) +
                            11 * id_FO) -
                           1])) {
          Fusion_Object_Assigned_Flag = 1;
          exitg1 = true;
        } else {
          //   Association_Map_Updated안에 id_FO 존재 여부 있으면
          //   Fusion_Object_Assigned_Flag =1; break;
          id_FST++;
        }
      }
      if (Fusion_Object_Assigned_Flag == 1) {
        for (i1 = 0; i1 < 24; i1++) {
          b_Fusion_Track_Updated[i1] =
              Fusion_Track_Updated[i1 + 24 * (static_cast<int>(t) - 1)];
        }
        loop_ub = Fusion_Track_out.size(0);
        for (i1 = 0; i1 < loop_ub; i1++) {
          Fusion_Track_out[i1 + Fusion_Track_out.size(0) *
                                    (static_cast<int>(t) - 1)] =
              b_Fusion_Track_Updated[i1];
        }
        b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
        for (i1 = 0; i1 < 6; i1++) {
          for (i2 = 0; i2 < 6; i2++) {
            P_Fusion_Track_Updated_tmp = i2 + 6 * i1;
            b_P_Fusion_Track_Updated[P_Fusion_Track_Updated_tmp] =
                P_Fusion_Track_Updated[P_Fusion_Track_Updated_tmp +
                                       36 * (static_cast<int>(t) - 1)];
          }
        }
        P_Fusion_Track_Updated_tmp = P_Fusion_Track_out.size(0);
        loop_ub = P_Fusion_Track_out.size(1);
        for (i1 = 0; i1 < loop_ub; i1++) {
          for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
            P_Fusion_Track_out[(i2 + P_Fusion_Track_out.size(0) * i1) +
                               P_Fusion_Track_out.size(0) *
                                   P_Fusion_Track_out.size(1) *
                                   (static_cast<int>(t) - 1)] =
                b_P_Fusion_Track_Updated[i2 + P_Fusion_Track_Updated_tmp * i1];
          }
        }
        //  updated_age +1
        P_Fusion_Track_Updated_tmp = 24 * (static_cast<int>(t) - 1);
        Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.UPDATED_AGE) +
                          Fusion_Track_out.size(0) *
                              (static_cast<int>(t) - 1)) -
                         1] =
            Fusion_Track_Updated[(static_cast<int>(
                                      FUSION_TRACK->TRACKING.UPDATED_AGE) +
                                  P_Fusion_Track_Updated_tmp) -
                                 1] +
            1.0;
        //  coasting_age -1
        d = Fusion_Track_Updated[(static_cast<int>(
                                      FUSION_TRACK->TRACKING.COASTING_AGE) +
                                  P_Fusion_Track_Updated_tmp) -
                                 1];
        if (d > 0.0) {
          Fusion_Track_out
              [(static_cast<int>(FUSION_TRACK->TRACKING.COASTING_AGE) +
                Fusion_Track_out.size(0) * (static_cast<int>(t) - 1)) -
               1] = d - 1.0;
        }
        //  life time +1
        Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.LIFE_TIME) +
                          Fusion_Track_out.size(0) *
                              (static_cast<int>(t) - 1)) -
                         1] =
            Fusion_Track_Updated[(static_cast<int>(
                                      FUSION_TRACK->TRACKING.LIFE_TIME) +
                                  P_Fusion_Track_Updated_tmp) -
                                 1] +
            1.0;
        if (Fusion_Track_out
                [(static_cast<int>(FUSION_TRACK->TRACKING.LIFE_TIME) +
                  Fusion_Track_out.size(0) * (static_cast<int>(t) - 1)) -
                 1] > TRACKING->THRESHOLD_MAX_LIFE_TIME) {
          //  3
          Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.LIFE_TIME) +
                            Fusion_Track_out.size(0) *
                                (static_cast<int>(t) - 1)) -
                           1] = TRACKING->THRESHOLD_MAX_LIFE_TIME;
        }
        Fusion_Object_Assigned_Flag = 0;
      }
    }
  }
  //  FST exist at k-1, measurement is not updated at k, then maintain FST track
  //  and life time -1
  for (id_FST = 0; id_FST < i; id_FST++) {
    d = Fusion_Track_Updated
        [(static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) + 24 * id_FST) -
         1];
    t = Fusion_Track_Updated
        [(static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) + 24 * id_FST) -
         1];
    if ((d * d + t * t != 0.0) && (Association_Map_Updated[id_FST] == 0.0) &&
        (Association_Map_out[id_FST] == 0.0)) {
      //          if Fusion_Object(FUSION_TRACK.MEASURE.LENGTH,id_FST) == 0
      Association_Map_out[id_FST] = Association_Map_k_1[id_FST];
      std::copy(&Fusion_Track_Updated[id_FST * 24],
                &Fusion_Track_Updated[static_cast<int>(id_FST * 24 + 24U)],
                &b_Fusion_Track_Updated[0]);
      loop_ub = Fusion_Track_out.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        Fusion_Track_out[i1 + Fusion_Track_out.size(0) * id_FST] =
            b_Fusion_Track_Updated[i1];
      }
      b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
      for (i1 = 0; i1 < 6; i1++) {
        for (i2 = 0; i2 < 6; i2++) {
          P_Fusion_Track_Updated_tmp = i2 + 6 * i1;
          b_P_Fusion_Track_Updated[P_Fusion_Track_Updated_tmp] =
              P_Fusion_Track_Updated[P_Fusion_Track_Updated_tmp + 36 * id_FST];
        }
      }
      P_Fusion_Track_Updated_tmp = P_Fusion_Track_out.size(0);
      loop_ub = P_Fusion_Track_out.size(1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
          P_Fusion_Track_out[(i2 + P_Fusion_Track_out.size(0) * i1) +
                             P_Fusion_Track_out.size(0) *
                                 P_Fusion_Track_out.size(1) * id_FST] =
              b_P_Fusion_Track_Updated[i2 + P_Fusion_Track_Updated_tmp * i1];
        }
      }
      //  updated_age -1
      Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.UPDATED_AGE) +
                        Fusion_Track_out.size(0) * id_FST) -
                       1] =
          Fusion_Track_Updated[(static_cast<int>(
                                    FUSION_TRACK->TRACKING.UPDATED_AGE) +
                                24 * id_FST) -
                               1] -
          1.0;
      //  coasting_age +1
      Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.COASTING_AGE) +
                        Fusion_Track_out.size(0) * id_FST) -
                       1] =
          Fusion_Track_Updated[(static_cast<int>(
                                    FUSION_TRACK->TRACKING.COASTING_AGE) +
                                24 * id_FST) -
                               1] +
          1.0;
      //  life time -1
      Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.LIFE_TIME) +
                        Fusion_Track_out.size(0) * id_FST) -
                       1] =
          Fusion_Track_Updated[(static_cast<int>(
                                    FUSION_TRACK->TRACKING.LIFE_TIME) +
                                24 * id_FST) -
                               1] -
          1.0;
      //              end
      //          end
    }
  }
  // --------------------------------------------------------------------------
  //  Track management - Creation
  // --------------------------------------------------------------------------
  //  create front vision track from unassigned front vision detection in
  //  Association_Map_Front_Vision ussasinged = if FVD is not assigned in
  //  Association_Map_Front_Vision_Updated, create new front vision track
  for (id_FO = 0; id_FO < i; id_FO++) {
    if (Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.LENGTH) +
                       11 * id_FO) -
                      1] != 0.0) {
      id_FST = 0;
      exitg1 = false;
      while ((!exitg1) &&
             (id_FST <= static_cast<int>(FUSION_TRACK->TRACK_NUMBER) - 1)) {
        if ((Association_Map_Updated[id_FST] != 0.0) &&
            (LIDAR_TRACK_SWITCH == DEFINITION->ON) &&
            (FRONT_VISION_TRACK_SWITCH == DEFINITION->OFF) &&
            (Fusion_Object[(static_cast<int>(
                                FUSION_TRACK->ASSOCIATION.FUSED_LDT_ID) +
                            11 * id_FO) -
                           1] == Association_Map_Updated[id_FST])) {
          Fusion_Object_Assigned_Flag = 1;
          exitg1 = true;
        } else {
          id_FST++;
        }
      }
      if (Fusion_Object_Assigned_Flag == 0) {
        //  Association_Map_Updated 빈칸 검색
        id_FST = 0;
        exitg1 = false;
        while ((!exitg1) &&
               (id_FST <= static_cast<int>(FUSION_TRACK->TRACK_NUMBER) - 1)) {
          if (Association_Map_out[id_FST] == 0.0) {
            //  Association_Map_Updated 를 update
            Association_Map_out[id_FST] = static_cast<double>(id_FST) + 1.0;
            // Fusion_Object(FUSION_TRACK.ASSOCIATION.FUSED_LDT_ID,id_FO);
            //  Fusion_Track_out 에다가 Fusion_Object 추가
            if (1.0 > FUSION_TRACK->MEASURE_STATE_NUMBER) {
              loop_ub = 0;
            } else {
              loop_ub = static_cast<int>(FUSION_TRACK->MEASURE_STATE_NUMBER);
            }
            for (i1 = 0; i1 < loop_ub; i1++) {
              Fusion_Track_out[i1 + Fusion_Track_out.size(0) * id_FST] =
                  Fusion_Object[i1 + 11 * id_FO];
            }
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.REL_POS_Y) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] =
                Fusion_Object[(static_cast<int>(
                                   FUSION_TRACK->MEASURE.REL_POS_Y) +
                               11 * id_FO) -
                              1];
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.REL_POS_X) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] =
                Fusion_Object[(static_cast<int>(
                                   FUSION_TRACK->MEASURE.REL_POS_X) +
                               11 * id_FO) -
                              1];
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.REL_VEL_Y) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] =
                Fusion_Object[(static_cast<int>(
                                   FUSION_TRACK->MEASURE.REL_VEL_Y) +
                               11 * id_FO) -
                              1];
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.REL_VEL_X) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] =
                Fusion_Object[(static_cast<int>(
                                   FUSION_TRACK->MEASURE.REL_VEL_X) +
                               11 * id_FO) -
                              1];
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->ASSOCIATION.FUSED_LDT_ID) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] = static_cast<double>(id_FST) + 1.0;
            // Fusion_Object(FUSION_TRACK.MEASURE.REL_VEL_X, id_FO);
            //  P_Fusion_Track_Updated
            if (TRACKING->FUSION_TRACK.STATE_NUMBER < 0.0) {
              P_Fusion_Track_Updated_tmp = 0;
              t = 0.0;
            } else {
              P_Fusion_Track_Updated_tmp =
                  static_cast<int>(TRACKING->FUSION_TRACK.STATE_NUMBER);
              t = TRACKING->FUSION_TRACK.STATE_NUMBER;
            }
            if (P_Fusion_Track_Updated_tmp <= static_cast<int>(t)) {
              Fusion_Object_Assigned_Flag = P_Fusion_Track_Updated_tmp;
            } else {
              Fusion_Object_Assigned_Flag = static_cast<int>(t);
            }
            b_I.set_size(P_Fusion_Track_Updated_tmp, static_cast<int>(t));
            loop_ub = P_Fusion_Track_Updated_tmp * static_cast<int>(t);
            for (i1 = 0; i1 < loop_ub; i1++) {
              b_I[i1] = 0;
            }
            if (Fusion_Object_Assigned_Flag > 0) {
              for (P_Fusion_Track_Updated_tmp = 0;
                   P_Fusion_Track_Updated_tmp < Fusion_Object_Assigned_Flag;
                   P_Fusion_Track_Updated_tmp++) {
                b_I[P_Fusion_Track_Updated_tmp +
                    b_I.size(0) * P_Fusion_Track_Updated_tmp] = 1;
              }
            }
            loop_ub = b_I.size(1);
            for (i1 = 0; i1 < loop_ub; i1++) {
              P_Fusion_Track_Updated_tmp = b_I.size(0);
              for (i2 = 0; i2 < P_Fusion_Track_Updated_tmp; i2++) {
                P_Fusion_Track_out[(i2 + P_Fusion_Track_out.size(0) * i1) +
                                   P_Fusion_Track_out.size(0) *
                                       P_Fusion_Track_out.size(1) * id_FST] =
                    b_I[i2 + b_I.size(0) * i1];
              }
            }
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.UPDATED_AGE) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] = 1.0;
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.COASTING_AGE) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] = 0.0;
            Fusion_Track_out[(static_cast<int>(
                                  FUSION_TRACK->TRACKING.LIFE_TIME) +
                              Fusion_Track_out.size(0) * id_FST) -
                             1] = TRACKING->DELETE_TRACK_COUNT;
            exitg1 = true;
          } else {
            id_FST++;
          }
        }
      }
      Fusion_Object_Assigned_Flag = 0;
    }
  }
  // --------------------------------------------------------------------------
  //  Track management - Deletion
  // --------------------------------------------------------------------------
  //
  //  if life time in front vision track is below
  //  threshold(TRACKING.FRONT_VISION.DELETE_TRACK_COUNT), delete the front
  //  vision track
  for (id_FST = 0; id_FST < i; id_FST++) {
    if ((Association_Map_out[id_FST] != 0.0) &&
        (Fusion_Track_out[(static_cast<int>(FUSION_TRACK->TRACKING.LIFE_TIME) +
                           Fusion_Track_out.size(0) * id_FST) -
                          1] < TRACKING->DELETE_TRACK_COUNT)) {
      loop_ub = Fusion_Track_out.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        Fusion_Track_out[i1 + Fusion_Track_out.size(0) * id_FST] = 0.0;
      }
      b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
      loop_ub = P_Fusion_Track_out.size(1);
      for (i1 = 0; i1 < loop_ub; i1++) {
        for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
          P_Fusion_Track_out[(i2 + P_Fusion_Track_out.size(0) * i1) +
                             P_Fusion_Track_out.size(0) *
                                 P_Fusion_Track_out.size(1) * id_FST] = 0.0;
        }
      }
      Association_Map_out[id_FST] = 0.0;
    }
  }
  //  % Delete overlapped fusion track (Vehicle)
  for (P_Fusion_Track_Updated_tmp = 0; P_Fusion_Track_Updated_tmp < i;
       P_Fusion_Track_Updated_tmp++) {
    if (Association_Map_out[P_Fusion_Track_Updated_tmp] != 0.0) {
      for (Fusion_Object_Assigned_Flag = 0; Fusion_Object_Assigned_Flag < i;
           Fusion_Object_Assigned_Flag++) {
        if ((static_cast<unsigned int>(P_Fusion_Track_Updated_tmp) !=
             static_cast<unsigned int>(Fusion_Object_Assigned_Flag)) &&
            (Association_Map_out[Fusion_Object_Assigned_Flag] != 0.0)) {
          double a;
          double b_a;
          a = Fusion_Track_out
              [(static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) +
                Fusion_Track_out.size(0) * P_Fusion_Track_Updated_tmp) -
               1];
          b_a = Fusion_Track_out
              [(static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) +
                Fusion_Track_out.size(0) * Fusion_Object_Assigned_Flag) -
               1];
          if (std::abs(a - b_a) < TRACKING->VEHICLE_GATING.X_MAX) {
            double c_a;
            double d_a;
            c_a = Fusion_Track_out
                [(static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) +
                  Fusion_Track_out.size(0) * P_Fusion_Track_Updated_tmp) -
                 1];
            d_a = Fusion_Track_out
                [(static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) +
                  Fusion_Track_out.size(0) * Fusion_Object_Assigned_Flag) -
                 1];
            if ((std::abs(c_a - d_a) < TRACKING->VEHICLE_GATING.Y_MAX) &&
                (LIDAR_TRACK_SWITCH == DEFINITION->ON) &&
                (FRONT_VISION_TRACK_SWITCH == DEFINITION->OFF)) {
              d = Fusion_Track_out
                  [(static_cast<int>(FUSION_TRACK->TRACKING.UPDATED_AGE) +
                    Fusion_Track_out.size(0) * P_Fusion_Track_Updated_tmp) -
                   1];
              t = Fusion_Track_out
                  [(static_cast<int>(FUSION_TRACK->TRACKING.UPDATED_AGE) +
                    Fusion_Track_out.size(0) * Fusion_Object_Assigned_Flag) -
                   1];
              if (d > t) {
                loop_ub = Fusion_Track_out.size(0);
                for (i1 = 0; i1 < loop_ub; i1++) {
                  Fusion_Track_out[i1 + Fusion_Track_out.size(0) *
                                            Fusion_Object_Assigned_Flag] = 0.0;
                }
                b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
                loop_ub = P_Fusion_Track_out.size(1);
                for (i1 = 0; i1 < loop_ub; i1++) {
                  for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
                    P_Fusion_Track_out[(i2 + P_Fusion_Track_out.size(0) * i1) +
                                       P_Fusion_Track_out.size(0) *
                                           P_Fusion_Track_out.size(1) *
                                           Fusion_Object_Assigned_Flag] = 0.0;
                  }
                }
                Association_Map_out[Fusion_Object_Assigned_Flag] = 0.0;
              } else if (d < t) {
                loop_ub = Fusion_Track_out.size(0);
                for (i1 = 0; i1 < loop_ub; i1++) {
                  Fusion_Track_out[i1 + Fusion_Track_out.size(0) *
                                            P_Fusion_Track_Updated_tmp] = 0.0;
                }
                b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
                loop_ub = P_Fusion_Track_out.size(1);
                for (i1 = 0; i1 < loop_ub; i1++) {
                  for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
                    P_Fusion_Track_out[(i2 + P_Fusion_Track_out.size(0) * i1) +
                                       P_Fusion_Track_out.size(0) *
                                           P_Fusion_Track_out.size(1) *
                                           P_Fusion_Track_Updated_tmp] = 0.0;
                  }
                }
                Association_Map_out[P_Fusion_Track_Updated_tmp] = 0.0;
              } else if (d == t) {
                if (std::sqrt(a * a + c_a * c_a) <
                    std::sqrt(b_a * b_a + d_a * d_a)) {
                  loop_ub = Fusion_Track_out.size(0);
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    Fusion_Track_out[i1 + Fusion_Track_out.size(0) *
                                              Fusion_Object_Assigned_Flag] =
                        0.0;
                  }
                  b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
                  loop_ub = P_Fusion_Track_out.size(1);
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
                      P_Fusion_Track_out[(i2 +
                                          P_Fusion_Track_out.size(0) * i1) +
                                         P_Fusion_Track_out.size(0) *
                                             P_Fusion_Track_out.size(1) *
                                             Fusion_Object_Assigned_Flag] = 0.0;
                    }
                  }
                  Association_Map_out[Fusion_Object_Assigned_Flag] = 0.0;
                } else {
                  loop_ub = Fusion_Track_out.size(0);
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    Fusion_Track_out[i1 + Fusion_Track_out.size(0) *
                                              P_Fusion_Track_Updated_tmp] = 0.0;
                  }
                  b_P_Fusion_Track_out = P_Fusion_Track_out.size(0);
                  loop_ub = P_Fusion_Track_out.size(1);
                  for (i1 = 0; i1 < loop_ub; i1++) {
                    for (i2 = 0; i2 < b_P_Fusion_Track_out; i2++) {
                      P_Fusion_Track_out[(i2 +
                                          P_Fusion_Track_out.size(0) * i1) +
                                         P_Fusion_Track_out.size(0) *
                                             P_Fusion_Track_out.size(1) *
                                             P_Fusion_Track_Updated_tmp] = 0.0;
                    }
                  }
                  Association_Map_out[P_Fusion_Track_Updated_tmp] = 0.0;
                }
              }
            }
          }
        }
      }
    }
  }
}

//
// File trailer for fusion_track_management.cpp
//
// [EOF]
//
