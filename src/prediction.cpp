//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: prediction.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 30-Nov-2021 15:28:02
//

// Include Files
#include "prediction.h"
#include "rt_nonfinite.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : const double Association_Map_k_1[32]
//                const double Fusion_Track_k_1[768]
//                const double P_Fusion_Track_k_1[1152]
//                double SAMPLE_TIME
//                const struct10_T *FUSION_TRACK
//                const struct17_T *TRACKING
//                coder::array<double, 2U> &Fusion_Track_Predicted
//                coder::array<double, 3U> &P_Fusion_Track_Predicted
// Return Type  : void
//
void prediction(const double Association_Map_k_1[32],
                const double Fusion_Track_k_1[768],
                const double P_Fusion_Track_k_1[1152], double SAMPLE_TIME,
                const struct10_T *FUSION_TRACK, const struct17_T *TRACKING,
                coder::array<double, 2U> &Fusion_Track_Predicted,
                coder::array<double, 3U> &P_Fusion_Track_Predicted)
{
  static const signed char iv[6]{0, 0, 0, 0, 1, 0};
  static const signed char iv1[6]{0, 0, 0, 0, 0, 1};
  double A_CA[36];
  double Q_CA[36];
  double b_A_CA[36];
  double c_A_CA[36];
  double b_Fusion_Track_k_1[24];
  double b[6];
  double A_CA_tmp;
  double A_CA_tmp_tmp;
  int b_FUSION_TRACK[6];
  int i;
  // --------------------------------------------------------------------------
  //  Fusion_Track_k_1 : [21 X 32]     Input
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
  //  11 : Fused Lidar Track ID
  //  12 : (Tracking) Relative position Y
  //  13 : (Tracking) Relative position X
  //  14 : (Tracking) Relative velocity Y
  //  15 : (Tracking) Relative velocity X
  //  16 : Updated Age
  //  17 : Coasting Age
  //  18 : Life time
  //  19 : Relative normal position with respect to lane
  //  20 : Relative tangential velocity with respect to lane
  //  21 : Relative normal velocity with respect to lane
  // --------------------------------------------------------------------------
  //  Association_Map_k_1 : [32 X 1]     Input
  // --------------------------------------------------------------------------
  //  1  : Lidar Track
  // --------------------------------------------------------------------------
  //  P_Fusion_Track_k_1 : [4 X 4 X 32]  Input
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  // --------------------------------------------------------------------------
  //  Fusion_Track_Predicted : [12 X 32]     Output
  // --------------------------------------------------------------------------
  //  1  : Relative position X
  //  2  : Relative position Y
  //  3  : Relative position Z
  //  4  : Relative velocity X
  //  5  : Relative velocity Y
  //  6  : Width
  //  7  : Height
  //  8  : Classification
  //  9  : Updated age
  //  10 : Life time
  //  11 : Range
  //  12 : Angle
  // --------------------------------------------------------------------------
  //  P_Front_Vision_Track_Predicted : [4 X 4 X 32]  Input
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  % Kalman Filter CV model parameter
  //  A_CV = [1 0 SAMPLE_TIME        0      % y
  //          0 1      0       SAMPLE_TIME  % x
  //          0 0      1             0      % vy
  //          0 0      0             1];    % vx
  //  Kalman Filter CA model parameter
  A_CA[0] = 1.0;
  A_CA[6] = 0.0;
  A_CA[12] = SAMPLE_TIME;
  A_CA[18] = 0.0;
  A_CA_tmp_tmp = SAMPLE_TIME * SAMPLE_TIME;
  A_CA_tmp = 0.5 * A_CA_tmp_tmp;
  A_CA[24] = A_CA_tmp;
  A_CA[30] = 0.0;
  A_CA[1] = 0.0;
  A_CA[7] = 1.0;
  A_CA[13] = 0.0;
  A_CA[19] = SAMPLE_TIME;
  A_CA[25] = 0.0;
  A_CA[31] = A_CA_tmp;
  A_CA[2] = 0.0;
  A_CA[8] = 0.0;
  A_CA[14] = 1.0;
  A_CA[20] = 0.0;
  A_CA[26] = SAMPLE_TIME;
  A_CA[32] = 0.0;
  A_CA[3] = 0.0;
  A_CA[9] = 0.0;
  A_CA[15] = 0.0;
  A_CA[21] = 1.0;
  A_CA[27] = 0.0;
  A_CA[33] = SAMPLE_TIME;
  for (i = 0; i < 6; i++) {
    A_CA[6 * i + 4] = iv[i];
    A_CA[6 * i + 5] = iv1[i];
  }
  double Q_CA_tmp;
  double Q_CA_tmp_tmp;
  double b_Q_CA_tmp;
  double b_Q_CA_tmp_tmp;
  double c_Q_CA_tmp;
  //  y
  //  x
  //  vy
  //  vx
  //  ay
  //  ax
  //  disturbance_y_CV = [1/2*SAMPLE_TIME^2 0 SAMPLE_TIME 0]';
  //  disturbance_x_CV = [0 1/2*SAMPLE_TIME^2 0 SAMPLE_TIME]';
  //  x_variance_CV = TRACKING.FUSION_TRACK.X_ACC_VAR_CV;
  //  y_variance_CV = TRACKING.FUSION_TRACK.Y_ACC_VAR_CV;
  //  Q_CV = disturbance_x_CV*x_variance_CV*disturbance_x_CV' +
  //  disturbance_y_CV*y_variance_CV*disturbance_y_CV'; Qx = 30*[SAMPLE_TIME^3/3
  //  SAMPLE_TIME^2/2; SAMPLE_TIME^2/2 SAMPLE_TIME]; Qy = 10*[SAMPLE_TIME^3/3
  //  SAMPLE_TIME^2/2; SAMPLE_TIME^2/2 SAMPLE_TIME]; % 횡방향 위치가 이
  //  모델에서는 종방향 보다 더 신뢰가 있다고 판단? Q_CV = blkdiag(Qx, Qy); Qx =
  //  3^2*[SAMPLE_TIME^4/4 SAMPLE_TIME^3/2 SAMPLE_TIME^2/2; SAMPLE_TIME^3/2
  //  SAMPLE_TIME^2 SAMPLE_TIME;SAMPLE_TIME^2/2 SAMPLE_TIME 1]; Qy =
  //  0.15^2*[SAMPLE_TIME^4/4 SAMPLE_TIME^3/2 SAMPLE_TIME^2/2; SAMPLE_TIME^3/2
  //  SAMPLE_TIME^2 SAMPLE_TIME;SAMPLE_TIME^2/2 SAMPLE_TIME 1]; Qy =
  //  10*[SAMPLE_TIME^5/20 SAMPLE_TIME^4/8 SAMPLE_TIME^3/6; SAMPLE_TIME^4/8
  //  SAMPLE_TIME^3/3 SAMPLE_TIME^2/2;SAMPLE_TIME^3/6 SAMPLE_TIME^2/2
  //  SAMPLE_TIME];
  std::memset(&Q_CA[0], 0, 36U * sizeof(double));
  Q_CA_tmp = rt_powd_snf(SAMPLE_TIME, 4.0);
  Q_CA[0] = TRACKING->FUSION_TRACK.X_ACC_VAR_CA * (Q_CA_tmp / 20.0);
  Q_CA_tmp_tmp = Q_CA_tmp / 8.0;
  Q_CA_tmp = TRACKING->FUSION_TRACK.X_ACC_VAR_CA * Q_CA_tmp_tmp;
  Q_CA[6] = Q_CA_tmp;
  b_Q_CA_tmp = rt_powd_snf(SAMPLE_TIME, 3.0);
  b_Q_CA_tmp_tmp = b_Q_CA_tmp / 6.0;
  c_Q_CA_tmp = TRACKING->FUSION_TRACK.X_ACC_VAR_CA * b_Q_CA_tmp_tmp;
  Q_CA[12] = c_Q_CA_tmp;
  Q_CA[1] = Q_CA_tmp;
  Q_CA_tmp = b_Q_CA_tmp / 3.0;
  Q_CA[7] = TRACKING->FUSION_TRACK.X_ACC_VAR_CA * Q_CA_tmp;
  A_CA_tmp = A_CA_tmp_tmp / 2.0;
  b_Q_CA_tmp = TRACKING->FUSION_TRACK.X_ACC_VAR_CA * A_CA_tmp;
  Q_CA[13] = b_Q_CA_tmp;
  Q_CA[2] = c_Q_CA_tmp;
  Q_CA[8] = b_Q_CA_tmp;
  Q_CA[14] = TRACKING->FUSION_TRACK.X_ACC_VAR_CA * SAMPLE_TIME;
  Q_CA[21] = TRACKING->FUSION_TRACK.Y_ACC_VAR_CA *
             (rt_powd_snf(SAMPLE_TIME, 5.0) / 20.0);
  b_Q_CA_tmp = TRACKING->FUSION_TRACK.Y_ACC_VAR_CA * Q_CA_tmp_tmp;
  Q_CA[27] = b_Q_CA_tmp;
  c_Q_CA_tmp = TRACKING->FUSION_TRACK.Y_ACC_VAR_CA * b_Q_CA_tmp_tmp;
  Q_CA[33] = c_Q_CA_tmp;
  Q_CA[22] = b_Q_CA_tmp;
  Q_CA[28] = TRACKING->FUSION_TRACK.Y_ACC_VAR_CA * Q_CA_tmp;
  Q_CA_tmp = TRACKING->FUSION_TRACK.Y_ACC_VAR_CA * A_CA_tmp;
  Q_CA[34] = Q_CA_tmp;
  Q_CA[23] = c_Q_CA_tmp;
  Q_CA[29] = Q_CA_tmp;
  Q_CA[35] = TRACKING->FUSION_TRACK.Y_ACC_VAR_CA * SAMPLE_TIME;
  //  disturbance_y_CA = [1/6*SAMPLE_TIME^3 0 1/2*SAMPLE_TIME^2 0 SAMPLE_TIME
  //  0]'; disturbance_x_CA = [0 1/6*SAMPLE_TIME^3 0 1/2*SAMPLE_TIME^2 0
  //  SAMPLE_TIME]'; x_accel_variance_CA = TRACKING.FUSION_TRACK.X_ACC_VAR_CA;
  //  y_accel_variance_CA = TRACKING.FUSION_TRACK.Y_ACC_VAR_CA;
  //  Q_CA = disturbance_x_CA*x_accel_variance_CA*disturbance_x_CA' +
  //  disturbance_y_CA*y_accel_variance_CA*disturbance_y_CA';
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  i = static_cast<int>(FUSION_TRACK->TRACK_NUMBER);
  Fusion_Track_Predicted.set_size(
      static_cast<int>(FUSION_TRACK->TRACKING_STATE_NUMBER), i);
  P_Fusion_Track_Predicted.set_size(
      static_cast<int>(TRACKING->FUSION_TRACK.STATE_NUMBER),
      static_cast<int>(TRACKING->FUSION_TRACK.STATE_NUMBER), i);
  //  Front_Vision_Track_Predicted =
  //  zeros(FRONT_VISION_TRACK.TRACKING_STATE_NUMBER,
  //  FRONT_VISION_TRACK.TRACK_NUMBER); P_Front_Vision_Track_Predicted =
  //  zeros(TRACKING.FRONT_VISION.STATE_NUMBER,
  //  TRACKING.FRONT_VISION.STATE_NUMBER, FRONT_VISION_TRACK.TRACK_NUMBER);
  // --------------------------------------------------------------------------
  //  Prediction(CV model)
  // --------------------------------------------------------------------------
  for (int id_FST{0}; id_FST < i; id_FST++) {
    int k;
    A_CA_tmp = Association_Map_k_1[0];
    for (k = 0; k < 31; k++) {
      A_CA_tmp += Association_Map_k_1[k + 1];
    }
    if (A_CA_tmp != 0.0) {
      int P_Fusion_Track_Predicted_idx_0;
      int b_P_Fusion_Track_Predicted;
      int i1;
      int i2;
      std::copy(&Fusion_Track_k_1[0], &Fusion_Track_k_1[24],
                &b_Fusion_Track_k_1[0]);
      k = Fusion_Track_Predicted.size(0);
      for (i1 = 0; i1 < k; i1++) {
        Fusion_Track_Predicted[i1] = b_Fusion_Track_k_1[i1];
      }
      b_FUSION_TRACK[0] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) - 1;
      b_FUSION_TRACK[1] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) - 1;
      b_FUSION_TRACK[2] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_Y) - 1;
      b_FUSION_TRACK[3] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_X) - 1;
      b_FUSION_TRACK[4] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_Y) - 1;
      b_FUSION_TRACK[5] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_X) - 1;
      for (i1 = 0; i1 < 6; i1++) {
        b[i1] = Fusion_Track_k_1[b_FUSION_TRACK[i1]];
      }
      b_FUSION_TRACK[0] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) - 1;
      b_FUSION_TRACK[1] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) - 1;
      b_FUSION_TRACK[2] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_Y) - 1;
      b_FUSION_TRACK[3] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_X) - 1;
      b_FUSION_TRACK[4] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_Y) - 1;
      b_FUSION_TRACK[5] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_X) - 1;
      b_P_Fusion_Track_Predicted = P_Fusion_Track_Predicted.size(0);
      for (i1 = 0; i1 < 6; i1++) {
        Fusion_Track_Predicted[b_FUSION_TRACK[i1]] = 0.0;
        i2 = b_FUSION_TRACK[i1];
        for (P_Fusion_Track_Predicted_idx_0 = 0;
             P_Fusion_Track_Predicted_idx_0 < 6;
             P_Fusion_Track_Predicted_idx_0++) {
          k = i1 + 6 * P_Fusion_Track_Predicted_idx_0;
          Fusion_Track_Predicted[i2] =
              Fusion_Track_Predicted[i2] +
              A_CA[k] * b[P_Fusion_Track_Predicted_idx_0];
          A_CA_tmp = 0.0;
          for (int i3{0}; i3 < 6; i3++) {
            A_CA_tmp +=
                A_CA[i1 + 6 * i3] *
                P_Fusion_Track_k_1[i3 + 6 * P_Fusion_Track_Predicted_idx_0];
          }
          c_A_CA[k] = A_CA_tmp;
        }
        for (i2 = 0; i2 < 6; i2++) {
          A_CA_tmp = 0.0;
          for (P_Fusion_Track_Predicted_idx_0 = 0;
               P_Fusion_Track_Predicted_idx_0 < 6;
               P_Fusion_Track_Predicted_idx_0++) {
            A_CA_tmp += c_A_CA[i1 + 6 * P_Fusion_Track_Predicted_idx_0] *
                        A_CA[i2 + 6 * P_Fusion_Track_Predicted_idx_0];
          }
          k = i1 + 6 * i2;
          b_A_CA[k] = A_CA_tmp + Q_CA[k];
        }
      }
      P_Fusion_Track_Predicted_idx_0 = P_Fusion_Track_Predicted.size(0);
      k = P_Fusion_Track_Predicted.size(1);
      for (i1 = 0; i1 < k; i1++) {
        for (i2 = 0; i2 < b_P_Fusion_Track_Predicted; i2++) {
          P_Fusion_Track_Predicted[i2 + P_Fusion_Track_Predicted.size(0) * i1] =
              b_A_CA[i2 + P_Fusion_Track_Predicted_idx_0 * i1];
        }
      }
      //          Fusion_Track_Predicted(FUSION_TRACK.MEASURE.WIDTH, id_FST) =
      //          Fusion_Track_k_1(FUSION_TRACK.MEASURE.WIDTH, id_FST);
      //          Fusion_Track_Predicted(FUSION_TRACK.MEASURE.LENGTH, id_FST) =
      //          Fusion_Track_k_1(FUSION_TRACK.MEASURE.LENGTH, id_FST);
      //          Fusion_Track_Predicted(FUSION_TRACK.MEASURE.HEIGHT, id_FST) =
      //          Fusion_Track_k_1(FUSION_TRACK.MEASURE.HEIGHT, id_FST);
      //          Fusion_Track_Predicted(FUSION_TRACK.MEASURE.HEADING_ANGLE,
      //          id_FST) = Fusion_Track_k_1(FUSION_TRACK.MEASURE.HEADING_ANGLE,
      //          id_FST);
      //
      //          Fusion_Track_Predicted(FUSION_TRACK.MEASURE.RANGE, id_FST) =
      //          Fusion_Track_k_1(FUSION_TRACK.MEASURE.RANGE, id_FST);
      //          Fusion_Track_Predicted(FUSION_TRACK.MEASURE.ANGLE, id_FST) =
      //          Fusion_Track_k_1(FUSION_TRACK.MEASURE.ANGLE, id_FST);
      //
      //          Fusion_Track_Predicted(FUSION_TRACK.ASSOCIATION.FUSED_LDT_ID,
      //          id_FST) =
      //          Fusion_Track_k_1(FUSION_TRACK.ASSOCIATION.FUSED_LDT_ID,
      //          id_FST);
      //
      //          Fusion_Track_Predicted(FUSION_TRACK.TRACKING.UPDATED_AGE,
      //          id_FST) = Fusion_Track_k_1(FUSION_TRACK.TRACKING.UPDATED_AGE,
      //          id_FST);
      //          Fusion_Track_Predicted(FUSION_TRACK.TRACKING.COASTING_AGE,
      //          id_FST) = Fusion_Track_k_1(FUSION_TRACK.TRACKING.COASTING_AGE,
      //          id_FST);
      //          Fusion_Track_Predicted(FUSION_TRACK.TRACKING.LIFE_TIME,
      //          id_FST) = Fusion_Track_k_1(FUSION_TRACK.TRACKING.LIFE_TIME,
      //          id_FST);
    }
  }
}

//
// File trailer for prediction.cpp
//
// [EOF]
//
