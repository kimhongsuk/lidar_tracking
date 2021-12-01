//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: correction.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 30-Nov-2021 15:28:02
//

// Include Files
#include "correction.h"
#include "mpower.h"
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "unit_conversion_types.h"
#include "coder_array.h"
#include <algorithm>
#include <cmath>

// Function Definitions
//
// Arguments    : const double Fusion_Object[352]
//                const double Association_Map_k_1[32]
//                const double Fusion_Track_Predicted[768]
//                const double P_Fusion_Track_Predicted[1152]
//                const struct10_T *FUSION_TRACK
//                const struct17_T *TRACKING
//                const struct27_T *ASSOCIATION
//                double Fusion_Track_Updated[768]
//                coder::array<double, 3U> &P_Fusion_Track_Updated
//                coder::array<double, 2U> &Association_Map_Updated
//                coder::array<double, 3U> &S_CA
//                coder::array<double, 2U> &Md_FV
// Return Type  : void
//
void correction(const double Fusion_Object[352],
                const double Association_Map_k_1[32],
                const double Fusion_Track_Predicted[768],
                const double P_Fusion_Track_Predicted[1152],
                const struct10_T *FUSION_TRACK, const struct17_T *TRACKING,
                const struct27_T *ASSOCIATION, double Fusion_Track_Updated[768],
                coder::array<double, 3U> &P_Fusion_Track_Updated,
                coder::array<double, 2U> &Association_Map_Updated,
                coder::array<double, 3U> &S_CA, coder::array<double, 2U> &Md_FV)
{
  coder::array<double, 3U> Conv_nu_Vehicle;
  coder::array<double, 3U> K_CA;
  coder::array<double, 3U> P_Fusion_Track_C;
  coder::array<double, 3U> P_Fusion_Track_tilda;
  coder::array<double, 3U> residual_FT;
  coder::array<double, 2U> B;
  coder::array<double, 2U> C;
  coder::array<double, 2U> b_C;
  coder::array<double, 2U> b_K_CA;
  coder::array<double, 2U> b_S_CA;
  coder::array<double, 2U> beta_FV_Vehicle;
  coder::array<double, 2U> beta_norm_Vehicle;
  coder::array<double, 2U> nu_Vehicle;
  coder::array<double, 1U> beta_norm_0_Vehicle;
  coder::array<double, 1U> sum_beta_FV_Vehicle;
  double b_TRACKING[36];
  double c_TRACKING[36];
  double y_tmp[36];
  double b[6];
  double b_y[6];
  double PD;
  double VEHICLE_GATING_THRESHOLD;
  double b_b;
  double s;
  double tmp_sum_beta;
  int b_FUSION_TRACK[6];
  int c_FUSION_TRACK[6];
  int b_i;
  int boffset;
  int coffset;
  int i;
  int i1;
  int id_FST;
  int j;
  int k;
  int y_tmp_tmp;
  bool exitg1;
  bool y;
  // --------------------------------------------------------------------------
  //  Fusion Object : [11 X 32]     Input
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
  //  Association_Map_k_1 : [32 X 1]     Input
  // --------------------------------------------------------------------------
  //  1  : Lidar Track
  // --------------------------------------------------------------------------
  //  Fusion_Track_Predicted : [12 X 32]     Input
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
  //  P_Fusion_Track_Predicted : [4 X 4 X 32]  Input
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  // --------------------------------------------------------------------------
  //  Fusion_Track_Updated : [21 X 32]     Output
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
  //  P_Fusion_Track_Updated : [4 X 4 X 32]  Output
  // --------------------------------------------------------------------------
  //  1  : Relative position Y
  //  2  : Relative position X
  //  3  : Relative velocity Y
  //  4  : Relative velocity X
  // --------------------------------------------------------------------------
  //  Association_Map_Updated : [32 X 1]     Output
  // --------------------------------------------------------------------------
  //  1  : Lidar Track
  // --------------------------------------------------------------------------
  //  Parameter
  // --------------------------------------------------------------------------
  //  Kalman Filter CV model parameter
  //  H_CV = TRACKING.LIDAR.H_CV ;
  //  R_CV = TRACKING.LIDAR.R_CV;
  //  PDAF parameter
  PD = TRACKING->FUSION_TRACK.PDAF.PD;
  //  target detection probability
  //  gate probability
  VEHICLE_GATING_THRESHOLD = TRACKING->FUSION_TRACK.VEHICLE_GATING_THRESHOLD;
  // --------------------------------------------------------------------------
  //  initialization
  // --------------------------------------------------------------------------
  i = static_cast<int>(FUSION_TRACK->TRACK_NUMBER);
  Association_Map_Updated.set_size(
      i, static_cast<int>(ASSOCIATION->SENSOR_NUMBER));
  // Fusion_Track_Updated =
  // coder.nullcopy(zeros(FUSION_TRACK.TRACKING_STATE_NUMBER,
  // FUSION_TRACK.TRACK_NUMBER));
  i1 = static_cast<int>(TRACKING->FUSION_TRACK.STATE_NUMBER);
  residual_FT.set_size(i1, i, i);
  //  y, x
  Md_FV.set_size(i, i);
  boffset = static_cast<int>(FUSION_TRACK->TRACK_NUMBER) *
            static_cast<int>(FUSION_TRACK->TRACK_NUMBER);
  for (k = 0; k < boffset; k++) {
    Md_FV[k] = TRACKING->FUSION_TRACK.MAHALANOBIS_DISTANCE.DEFAULT_VALUE;
  }
  beta_FV_Vehicle.set_size(i, i);
  sum_beta_FV_Vehicle.set_size(i);
  beta_norm_Vehicle.set_size(i, i);
  beta_norm_0_Vehicle.set_size(i);
  nu_Vehicle.set_size(i1, i);
  Conv_nu_Vehicle.set_size(i1, i1, i);
  P_Fusion_Track_Updated.set_size(i1, i1, i);
  P_Fusion_Track_C.set_size(i1, i1, i);
  P_Fusion_Track_tilda.set_size(i1, i1, i);
  S_CA.set_size(i1, i1, i);
  K_CA.set_size(i1, i1, i);
  std::copy(&Fusion_Track_Predicted[0], &Fusion_Track_Predicted[768],
            &Fusion_Track_Updated[0]);
  //  if LIDAR_TRACK_SWITCH == DEFINITION.ON
  // --------------------------------------------------------------------------
  //  Compute innovation covariance matrix and kalman gain
  // --------------------------------------------------------------------------
  for (id_FST = 0; id_FST < i; id_FST++) {
    s = Association_Map_k_1[0];
    for (k = 0; k < 31; k++) {
      s += Association_Map_k_1[k + 1];
    }
    if (s != 0.0) {
      for (k = 0; k < 6; k++) {
        for (j = 0; j < 6; j++) {
          y_tmp_tmp = k + 6 * j;
          y_tmp[j + 6 * k] = TRACKING->LIDAR.H_CA[y_tmp_tmp];
          s = 0.0;
          for (boffset = 0; boffset < 6; boffset++) {
            s += TRACKING->LIDAR.H_CA[k + 6 * boffset] *
                 P_Fusion_Track_Predicted[boffset + 6 * j];
          }
          b_TRACKING[y_tmp_tmp] = s;
        }
      }
      coffset = S_CA.size(0);
      for (k = 0; k < 6; k++) {
        for (j = 0; j < 6; j++) {
          s = 0.0;
          for (boffset = 0; boffset < 6; boffset++) {
            s += b_TRACKING[k + 6 * boffset] * y_tmp[boffset + 6 * j];
          }
          y_tmp_tmp = k + 6 * j;
          c_TRACKING[y_tmp_tmp] = s + TRACKING->LIDAR.R_CA[y_tmp_tmp];
        }
      }
      y_tmp_tmp = S_CA.size(0);
      boffset = S_CA.size(1);
      for (k = 0; k < boffset; k++) {
        for (j = 0; j < coffset; j++) {
          S_CA[j + S_CA.size(0) * k] = c_TRACKING[j + y_tmp_tmp * k];
        }
      }
      for (k = 0; k < 6; k++) {
        for (j = 0; j < 6; j++) {
          s = 0.0;
          for (boffset = 0; boffset < 6; boffset++) {
            s += P_Fusion_Track_Predicted[k + 6 * boffset] *
                 y_tmp[boffset + 6 * j];
          }
          b_TRACKING[k + 6 * j] = s;
        }
      }
      boffset = S_CA.size(0);
      y_tmp_tmp = S_CA.size(1);
      b_S_CA.set_size(S_CA.size(0), S_CA.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_S_CA[j + b_S_CA.size(0) * k] = S_CA[j + S_CA.size(0) * k];
        }
      }
      coder::mpower(b_S_CA, B);
      y_tmp_tmp = B.size(1);
      C.set_size(6, B.size(1));
      for (j = 0; j < y_tmp_tmp; j++) {
        coffset = j * 6;
        boffset = j * B.size(0);
        for (b_i = 0; b_i < 6; b_i++) {
          s = 0.0;
          for (k = 0; k < 6; k++) {
            s += b_TRACKING[k * 6 + b_i] * B[boffset + k];
          }
          C[coffset + b_i] = s;
        }
      }
      boffset = C.size(1);
      for (k = 0; k < boffset; k++) {
        for (j = 0; j < 6; j++) {
          K_CA[j + K_CA.size(0) * k] = C[j + 6 * k];
        }
      }
    }
  }
  // --------------------------------------------------------------------------
  //  Gating for vehicle
  // --------------------------------------------------------------------------
  for (id_FST = 0; id_FST < i; id_FST++) {
    s = Association_Map_k_1[0];
    for (k = 0; k < 31; k++) {
      s += Association_Map_k_1[k + 1];
    }
    if (s != 0.0) {
      for (b_i = 0; b_i < i; b_i++) {
        s = Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.REL_POS_X) +
                           11 * b_i) -
                          1];
        tmp_sum_beta =
            Fusion_Object[(static_cast<int>(FUSION_TRACK->MEASURE.REL_POS_Y) +
                           11 * b_i) -
                          1];
        if (s * s + tmp_sum_beta * tmp_sum_beta != 0.0) {
          b_FUSION_TRACK[0] =
              static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) - 1;
          b_FUSION_TRACK[1] =
              static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) - 1;
          b_FUSION_TRACK[2] =
              static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_X) - 1;
          b_FUSION_TRACK[3] =
              static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_Y) - 1;
          b_FUSION_TRACK[4] =
              static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_X) - 1;
          b_FUSION_TRACK[5] =
              static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_Y) - 1;
          for (k = 0; k < 6; k++) {
            b[k] = Fusion_Track_Predicted[b_FUSION_TRACK[k]];
          }
          for (k = 0; k < 6; k++) {
            b_b = 0.0;
            for (j = 0; j < 6; j++) {
              b_b += TRACKING->LIDAR.H_CA[k + 6 * j] * b[j];
            }
            b_y[k] = b_b;
          }
          b[0] = s - b_y[0];
          b[1] = tmp_sum_beta - b_y[1];
          b[2] = 0.0 - b_y[2];
          b[3] = 0.0 - b_y[3];
          b[4] = 0.0 - b_y[4];
          b[5] = 0.0 - b_y[5];
          boffset = residual_FT.size(0);
          for (k = 0; k < boffset; k++) {
            residual_FT[k + residual_FT.size(0) * residual_FT.size(1) * b_i] =
                b[k];
          }
          boffset = S_CA.size(0);
          y_tmp_tmp = S_CA.size(1);
          b_S_CA.set_size(S_CA.size(0), S_CA.size(1));
          for (k = 0; k < y_tmp_tmp; k++) {
            for (j = 0; j < boffset; j++) {
              b_S_CA[j + b_S_CA.size(0) * k] = S_CA[j + S_CA.size(0) * k];
            }
          }
          coder::mpower(b_S_CA, B);
          y_tmp_tmp = residual_FT.size(0) - 1;
          coffset = B.size(1);
          b_C.set_size(1, B.size(1));
          for (j = 0; j < coffset; j++) {
            boffset = j * B.size(0);
            b_C[j] = 0.0;
            for (k = 0; k <= y_tmp_tmp; k++) {
              b_C[j] = b_C[j] + residual_FT[k + residual_FT.size(0) *
                                                    residual_FT.size(1) * b_i] *
                                    B[boffset + k];
            }
          }
          s = 0.0;
          boffset = b_C.size(1);
          for (k = 0; k < boffset; k++) {
            s += b_C[k] * residual_FT[k + residual_FT.size(0) *
                                              residual_FT.size(1) * b_i];
          }
          Md_FV[Md_FV.size(0) * b_i] = s;
          if (s <= VEHICLE_GATING_THRESHOLD) {
            //                          Association_Map_Updated(id_FST, :) =
            //                          id_FO;
            boffset = Association_Map_Updated.size(1);
            for (k = 0; k < boffset; k++) {
              Association_Map_Updated[Association_Map_Updated.size(0) * k] =
                  Fusion_Object[(static_cast<int>(
                                     FUSION_TRACK->ASSOCIATION.FUSED_LDT_ID) +
                                 11 * b_i) -
                                1];
            }
            Fusion_Track_Updated[static_cast<int>(
                                     FUSION_TRACK->ASSOCIATION.FUSED_LDT_ID) -
                                 1] = 1.0;
            beta_FV_Vehicle[beta_FV_Vehicle.size(0) * b_i] =
                std::exp(-0.5 * Md_FV[Md_FV.size(0) * b_i]);
          }
        }
      }
    }
  }
  // --------------------------------------------------------------------------
  //  Correction for Vehicle
  // --------------------------------------------------------------------------
  for (id_FST = 0; id_FST < i; id_FST++) {
    tmp_sum_beta = 0.0;
    s = Association_Map_k_1[0];
    for (k = 0; k < 31; k++) {
      s += Association_Map_k_1[k + 1];
    }
    if (s != 0.0) {
      for (b_i = 0; b_i < i; b_i++) {
        y = true;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k < 32)) {
          if (!(Association_Map_k_1[k] != 0.0)) {
            y = false;
            exitg1 = true;
          } else {
            k++;
          }
        }
        if (y) {
          tmp_sum_beta += beta_FV_Vehicle[beta_FV_Vehicle.size(0) * b_i];
        }
      }
      sum_beta_FV_Vehicle[0] = tmp_sum_beta;
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Gating%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //  refer) Bayesian Environment Representation, Prediction, and
      //  Criticality Assessment for Driver Assistance Systems
      //          b =
      //          2*TRACKING.FUSION_TRACK.STATE_NUMBER*(1-PD*PG)/(VEHICLE_GATING_THRESHOLD*PD);
      //          % back up code , autoware beta_norm_Vehicle(id_FST,:) =
      //          beta_FV_Vehicle(id_FST,:) ./ (1 - PD*PG +
      //          sum_beta_FV_Vehicle(id_FST,1)); beta_norm_0_Vehicle(id_FST,1)
      //          = (1 - PG*PD) / (1 - PD*PG + sum_beta_FV_Vehicle(id_FST,1));
      // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Gating%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      //  refer) 3D LiDAR Object Tracking for Autonomous driving
      //          Vu = pi;
      //          Vk = Vu*sqrt(det(VEHICLE_GATING_THRESHOLD*S_CA(:,:,id_FST)));
      //          b = 2*pi*TRACKING.FUSION_TRACK.STATE_NUMBER*(1-PD*PG)/(Vk*PD);
      b_b = 4.0 * (1.0 - PD * TRACKING->FUSION_TRACK.PDAF.PG) /
            (VEHICLE_GATING_THRESHOLD * PD);
      //  %         beta_norm_Vehicle(id_FST,:) = beta_FV_Vehicle(id_FST,:) ./
      //  (1 - PD*PG + sum_beta_FV_Vehicle(id_FST,1));
      s = b_b + tmp_sum_beta;
      boffset = beta_FV_Vehicle.size(1);
      for (k = 0; k < boffset; k++) {
        beta_norm_Vehicle[beta_norm_Vehicle.size(0) * k] =
            beta_FV_Vehicle[beta_FV_Vehicle.size(0) * k] / s;
      }
      //  %         beta_norm_0_Vehicle(id_FST,1) = (1 - PG*PD) / (1 - PD*PG +
      //  sum_beta_FV_Vehicle(id_FST,1));
      beta_norm_0_Vehicle[0] = b_b / (b_b + sum_beta_FV_Vehicle[0]);
    }
  }
  for (id_FST = 0; id_FST < i; id_FST++) {
    s = Association_Map_k_1[0];
    for (k = 0; k < 31; k++) {
      s += Association_Map_k_1[k + 1];
    }
    if (s != 0.0) {
      sum_beta_FV_Vehicle.set_size(i1);
      beta_FV_Vehicle.set_size(i1, i1);
      for (b_i = 0; b_i < i; b_i++) {
        y = true;
        k = 0;
        exitg1 = false;
        while ((!exitg1) && (k < 32)) {
          if (!(Association_Map_k_1[k] != 0.0)) {
            y = false;
            exitg1 = true;
          } else {
            k++;
          }
        }
        if (y) {
          s = beta_norm_Vehicle[beta_norm_Vehicle.size(0) * b_i];
          boffset = sum_beta_FV_Vehicle.size(0);
          for (k = 0; k < boffset; k++) {
            sum_beta_FV_Vehicle[k] =
                sum_beta_FV_Vehicle[k] +
                s * residual_FT[k + residual_FT.size(0) * residual_FT.size(1) *
                                        b_i];
          }
          s = beta_norm_Vehicle[beta_norm_Vehicle.size(0) * b_i];
          boffset = residual_FT.size(0);
          beta_FV_Vehicle.set_size(residual_FT.size(0), residual_FT.size(0));
          for (k = 0; k < boffset; k++) {
            for (j = 0; j < boffset; j++) {
              beta_FV_Vehicle[j + beta_FV_Vehicle.size(0) * k] =
                  beta_FV_Vehicle[j + beta_FV_Vehicle.size(0) * k] +
                  s *
                      residual_FT[j + residual_FT.size(0) *
                                          residual_FT.size(1) * b_i] *
                      residual_FT[k + residual_FT.size(0) *
                                          residual_FT.size(1) * b_i];
            }
          }
        }
      }
      boffset = sum_beta_FV_Vehicle.size(0);
      for (k = 0; k < boffset; k++) {
        nu_Vehicle[k] = sum_beta_FV_Vehicle[k];
      }
      boffset = beta_FV_Vehicle.size(1);
      for (k = 0; k < boffset; k++) {
        y_tmp_tmp = beta_FV_Vehicle.size(0);
        for (j = 0; j < y_tmp_tmp; j++) {
          Conv_nu_Vehicle[j + Conv_nu_Vehicle.size(0) * k] =
              beta_FV_Vehicle[j + beta_FV_Vehicle.size(0) * k];
        }
      }
      boffset = K_CA.size(0);
      y_tmp_tmp = K_CA.size(1);
      b_K_CA.set_size(K_CA.size(0), K_CA.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_K_CA[j + b_K_CA.size(0) * k] = K_CA[j + K_CA.size(0) * k];
        }
      }
      boffset = S_CA.size(0);
      y_tmp_tmp = S_CA.size(1);
      b_S_CA.set_size(S_CA.size(0), S_CA.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_S_CA[j + b_S_CA.size(0) * k] = S_CA[j + S_CA.size(0) * k];
        }
      }
      coder::internal::blas::mtimes(b_K_CA, b_S_CA, beta_FV_Vehicle);
      boffset = K_CA.size(0);
      y_tmp_tmp = K_CA.size(1);
      b_K_CA.set_size(K_CA.size(0), K_CA.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_K_CA[j + b_K_CA.size(0) * k] = K_CA[j + K_CA.size(0) * k];
        }
      }
      coder::internal::blas::b_mtimes(beta_FV_Vehicle, b_K_CA, B);
      coffset = P_Fusion_Track_C.size(0);
      for (k = 0; k < 6; k++) {
        for (j = 0; j < 6; j++) {
          y_tmp_tmp = j + 6 * k;
          b_TRACKING[y_tmp_tmp] =
              P_Fusion_Track_Predicted[y_tmp_tmp] - B[y_tmp_tmp];
        }
      }
      y_tmp_tmp = P_Fusion_Track_C.size(0);
      boffset = P_Fusion_Track_C.size(1);
      for (k = 0; k < boffset; k++) {
        for (j = 0; j < coffset; j++) {
          P_Fusion_Track_C[j + P_Fusion_Track_C.size(0) * k] =
              b_TRACKING[j + y_tmp_tmp * k];
        }
      }
      boffset = K_CA.size(0);
      y_tmp_tmp = K_CA.size(1);
      b_K_CA.set_size(K_CA.size(0), K_CA.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_K_CA[j + b_K_CA.size(0) * k] = K_CA[j + K_CA.size(0) * k];
        }
      }
      boffset = nu_Vehicle.size(0);
      b_S_CA.set_size(nu_Vehicle.size(0), nu_Vehicle.size(0));
      for (k = 0; k < boffset; k++) {
        for (j = 0; j < boffset; j++) {
          b_S_CA[j + b_S_CA.size(0) * k] =
              Conv_nu_Vehicle[j + Conv_nu_Vehicle.size(0) * k] -
              nu_Vehicle[j] * nu_Vehicle[k];
        }
      }
      coder::internal::blas::mtimes(b_K_CA, b_S_CA, beta_FV_Vehicle);
      boffset = K_CA.size(0);
      y_tmp_tmp = K_CA.size(1);
      b_K_CA.set_size(K_CA.size(0), K_CA.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_K_CA[j + b_K_CA.size(0) * k] = K_CA[j + K_CA.size(0) * k];
        }
      }
      coder::internal::blas::b_mtimes(beta_FV_Vehicle, b_K_CA, B);
      boffset = B.size(1);
      for (k = 0; k < boffset; k++) {
        y_tmp_tmp = B.size(0);
        for (j = 0; j < y_tmp_tmp; j++) {
          P_Fusion_Track_tilda[j + P_Fusion_Track_tilda.size(0) * k] =
              B[j + B.size(0) * k];
        }
      }
      s = beta_norm_0_Vehicle[0];
      boffset = P_Fusion_Track_C.size(0);
      y_tmp_tmp = P_Fusion_Track_C.size(1);
      B.set_size(P_Fusion_Track_C.size(0), P_Fusion_Track_C.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          B[j + B.size(0) * k] =
              (1.0 - s) * P_Fusion_Track_C[j + P_Fusion_Track_C.size(0) * k];
        }
      }
      boffset = P_Fusion_Track_tilda.size(0);
      y_tmp_tmp = P_Fusion_Track_tilda.size(1);
      b_S_CA.set_size(P_Fusion_Track_tilda.size(0),
                      P_Fusion_Track_tilda.size(1));
      for (k = 0; k < y_tmp_tmp; k++) {
        for (j = 0; j < boffset; j++) {
          b_S_CA[j + b_S_CA.size(0) * k] =
              P_Fusion_Track_tilda[j + P_Fusion_Track_tilda.size(0) * k];
        }
      }
      coffset = P_Fusion_Track_Updated.size(0);
      for (k = 0; k < 6; k++) {
        for (j = 0; j < 6; j++) {
          y_tmp_tmp = j + 6 * k;
          b_TRACKING[y_tmp_tmp] =
              (s * P_Fusion_Track_Predicted[y_tmp_tmp] + B[y_tmp_tmp]) +
              b_S_CA[y_tmp_tmp];
        }
      }
      y_tmp_tmp = P_Fusion_Track_Updated.size(0);
      boffset = P_Fusion_Track_Updated.size(1);
      for (k = 0; k < boffset; k++) {
        for (j = 0; j < coffset; j++) {
          P_Fusion_Track_Updated[j + P_Fusion_Track_Updated.size(0) * k] =
              b_TRACKING[j + y_tmp_tmp * k];
        }
      }
      b_FUSION_TRACK[0] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) - 1;
      b_FUSION_TRACK[1] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) - 1;
      b_FUSION_TRACK[2] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_X) - 1;
      b_FUSION_TRACK[3] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_Y) - 1;
      b_FUSION_TRACK[4] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_X) - 1;
      b_FUSION_TRACK[5] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_Y) - 1;
      c_FUSION_TRACK[0] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_X) - 1;
      c_FUSION_TRACK[1] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_POS_Y) - 1;
      c_FUSION_TRACK[2] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_X) - 1;
      c_FUSION_TRACK[3] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_VEL_Y) - 1;
      c_FUSION_TRACK[4] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_X) - 1;
      c_FUSION_TRACK[5] =
          static_cast<int>(FUSION_TRACK->TRACKING.REL_ACC_Y) - 1;
      for (k = 0; k < 6; k++) {
        Fusion_Track_Updated[c_FUSION_TRACK[k]] =
            Fusion_Track_Predicted[b_FUSION_TRACK[k]];
      }
    }
  }
  //  end
}

//
// File trailer for correction.cpp
//
// [EOF]
//
