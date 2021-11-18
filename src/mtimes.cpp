//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mtimes.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Nov-2021 11:27:25
//

// Include Files
#include "mtimes.h"
#include "rt_nonfinite.h"
#include "coder_array.h"

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 2U> &B
//                ::coder::array<double, 2U> &C
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void b_mtimes(const ::coder::array<double, 2U> &A,
              const ::coder::array<double, 2U> &B,
              ::coder::array<double, 2U> &C)
{
  int inner;
  int mc;
  int nc;
  mc = A.size(0);
  inner = A.size(1);
  nc = B.size(0);
  C.set_size(A.size(0), B.size(0));
  for (int j{0}; j < nc; j++) {
    int coffset;
    int i;
    coffset = j * mc;
    for (i = 0; i < mc; i++) {
      C[coffset + i] = 0.0;
    }
    for (int k{0}; k < inner; k++) {
      double bkj;
      int aoffset;
      aoffset = k * A.size(0);
      bkj = B[k * B.size(0) + j];
      for (i = 0; i < mc; i++) {
        int b_i;
        b_i = coffset + i;
        C[b_i] = C[b_i] + A[aoffset + i] * bkj;
      }
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &A
//                const ::coder::array<double, 2U> &B
//                ::coder::array<double, 2U> &C
// Return Type  : void
//
void mtimes(const ::coder::array<double, 2U> &A,
            const ::coder::array<double, 2U> &B, ::coder::array<double, 2U> &C)
{
  int inner;
  int mc;
  int nc;
  mc = A.size(0);
  inner = A.size(1);
  nc = B.size(1);
  C.set_size(A.size(0), B.size(1));
  for (int j{0}; j < nc; j++) {
    int boffset;
    int coffset;
    int i;
    coffset = j * mc;
    boffset = j * B.size(0);
    for (i = 0; i < mc; i++) {
      C[coffset + i] = 0.0;
    }
    for (int k{0}; k < inner; k++) {
      double bkj;
      int aoffset;
      aoffset = k * A.size(0);
      bkj = B[boffset + k];
      for (i = 0; i < mc; i++) {
        int b_i;
        b_i = coffset + i;
        C[b_i] = C[b_i] + A[aoffset + i] * bkj;
      }
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for mtimes.cpp
//
// [EOF]
//
