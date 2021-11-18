//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpower.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 18-Nov-2021 11:27:25
//

// Include Files
#include "mpower.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &a
//                ::coder::array<double, 2U> &c
// Return Type  : void
//
namespace coder {
void mpower(const ::coder::array<double, 2U> &a, ::coder::array<double, 2U> &c)
{
  array<double, 2U> x;
  array<int, 2U> ipiv;
  array<int, 2U> p;
  if ((a.size(0) == 0) || (a.size(1) == 0)) {
    int b_n;
    c.set_size(a.size(0), a.size(1));
    b_n = a.size(0) * a.size(1);
    for (int i{0}; i < b_n; i++) {
      c[i] = a[i];
    }
  } else {
    int b_i;
    int b_n;
    int i;
    int i1;
    int j;
    int k;
    int ldap1;
    int n;
    int u1;
    int yk;
    n = a.size(0);
    c.set_size(a.size(0), a.size(1));
    b_n = a.size(0) * a.size(1);
    for (i = 0; i < b_n; i++) {
      c[i] = 0.0;
    }
    x.set_size(a.size(0), a.size(1));
    b_n = a.size(0) * a.size(1);
    for (i = 0; i < b_n; i++) {
      x[i] = a[i];
    }
    b_n = a.size(0);
    ipiv.set_size(1, a.size(0));
    ipiv[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      ipiv[k - 1] = yk;
    }
    ldap1 = a.size(0);
    b_n = a.size(0) - 1;
    u1 = a.size(0);
    if (b_n < u1) {
      u1 = b_n;
    }
    for (j = 0; j < u1; j++) {
      double smax;
      int jj;
      int jp1j;
      int mmj_tmp;
      mmj_tmp = n - j;
      yk = j * (n + 1);
      jj = j * (ldap1 + 1);
      jp1j = yk + 2;
      if (mmj_tmp < 1) {
        b_n = -1;
      } else {
        b_n = 0;
        if (mmj_tmp > 1) {
          smax = std::abs(x[jj]);
          for (k = 2; k <= mmj_tmp; k++) {
            double s;
            s = std::abs(x[(yk + k) - 1]);
            if (s > smax) {
              b_n = k - 1;
              smax = s;
            }
          }
        }
      }
      if (x[jj + b_n] != 0.0) {
        if (b_n != 0) {
          b_n += j;
          ipiv[j] = b_n + 1;
          for (k = 0; k < n; k++) {
            smax = x[j + k * n];
            x[j + k * n] = x[b_n + k * n];
            x[b_n + k * n] = smax;
          }
        }
        i = jj + mmj_tmp;
        for (b_i = jp1j; b_i <= i; b_i++) {
          x[b_i - 1] = x[b_i - 1] / x[jj];
        }
      }
      b_n = yk + n;
      yk = jj + ldap1;
      for (jp1j = 0; jp1j <= mmj_tmp - 2; jp1j++) {
        smax = x[b_n + jp1j * n];
        if (x[b_n + jp1j * n] != 0.0) {
          i = yk + 2;
          i1 = mmj_tmp + yk;
          for (b_i = i; b_i <= i1; b_i++) {
            x[b_i - 1] = x[b_i - 1] + x[((jj + b_i) - yk) - 1] * -smax;
          }
        }
        yk += n;
      }
    }
    b_n = a.size(0);
    p.set_size(1, a.size(0));
    p[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      p[k - 1] = yk;
    }
    i = ipiv.size(1);
    for (k = 0; k < i; k++) {
      i1 = ipiv[k];
      if (i1 > k + 1) {
        b_n = p[i1 - 1];
        p[i1 - 1] = p[k];
        p[k] = b_n;
      }
    }
    for (k = 0; k < n; k++) {
      i = p[k];
      c[k + c.size(0) * (i - 1)] = 1.0;
      for (j = k + 1; j <= n; j++) {
        if (c[(j + c.size(0) * (i - 1)) - 1] != 0.0) {
          i1 = j + 1;
          for (b_i = i1; b_i <= n; b_i++) {
            c[(b_i + c.size(0) * (i - 1)) - 1] =
                c[(b_i + c.size(0) * (i - 1)) - 1] -
                c[(j + c.size(0) * (i - 1)) - 1] *
                    x[(b_i + x.size(0) * (j - 1)) - 1];
          }
        }
      }
    }
    for (j = 0; j < n; j++) {
      b_n = n * j - 1;
      for (k = n; k >= 1; k--) {
        yk = n * (k - 1) - 1;
        i = k + b_n;
        if (c[i] != 0.0) {
          c[i] = c[i] / x[k + yk];
          for (b_i = 0; b_i <= k - 2; b_i++) {
            i1 = (b_i + b_n) + 1;
            c[i1] = c[i1] - c[i] * x[(b_i + yk) + 1];
          }
        }
      }
    }
  }
}

} // namespace coder

//
// File trailer for mpower.cpp
//
// [EOF]
//
