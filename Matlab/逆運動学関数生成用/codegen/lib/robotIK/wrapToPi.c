/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: wrapToPi.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "wrapToPi.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double theta_data[]
 *                int theta_size[2]
 * Return Type  : void
 */
void wrapToPi(double theta_data[], int theta_size[2])
{
  double y_data[192];
  int i;
  int k;
  int loop_ub;
  int nx_tmp;
  int y_size_idx_0;
  int y_size_idx_1;
  unsigned char b_tmp_data[192];
  bool pos_data[192];
  bool tmp_data[192];
  bool exitg1;
  bool varargout_1;
  nx_tmp = theta_size[0] * theta_size[1];
  y_size_idx_0 = (signed char)theta_size[0];
  y_size_idx_1 = (signed char)theta_size[1];
  for (k = 0; k < nx_tmp; k++) {
    y_data[k] = fabs(theta_data[k]);
  }
  loop_ub = y_size_idx_0 * y_size_idx_1;
  for (i = 0; i < loop_ub; i++) {
    pos_data[i] = (y_data[i] > 3.1415926535897931);
  }
  varargout_1 = false;
  k = 1;
  exitg1 = false;
  while ((!exitg1) && (k <= y_size_idx_0 * y_size_idx_1)) {
    if (!pos_data[k - 1]) {
      k++;
    } else {
      varargout_1 = true;
      exitg1 = true;
    }
  }
  if (varargout_1) {
    y_size_idx_0 = theta_size[0];
    y_size_idx_1 = theta_size[1];
    for (i = 0; i < nx_tmp; i++) {
      y_data[i] = theta_data[i] + 3.1415926535897931;
    }
    loop_ub = y_size_idx_0 * y_size_idx_1;
    for (i = 0; i < loop_ub; i++) {
      pos_data[i] = (y_data[i] > 0.0);
    }
    theta_size[0] = (signed char)y_size_idx_0;
    theta_size[1] = (signed char)y_size_idx_1;
    nx_tmp = theta_size[0] * theta_size[1];
    for (k = 0; k < nx_tmp; k++) {
      theta_data[k] = b_mod(y_data[k]);
    }
    for (i = 0; i < nx_tmp; i++) {
      tmp_data[i] = (theta_data[i] == 0.0);
    }
    k = theta_size[0] * theta_size[1] - 1;
    nx_tmp = 0;
    y_size_idx_0 = 0;
    for (y_size_idx_1 = 0; y_size_idx_1 <= k; y_size_idx_1++) {
      if (tmp_data[y_size_idx_1] && pos_data[y_size_idx_1]) {
        nx_tmp++;
        b_tmp_data[y_size_idx_0] = (unsigned char)(y_size_idx_1 + 1);
        y_size_idx_0++;
      }
    }
    loop_ub = nx_tmp - 1;
    for (i = 0; i <= loop_ub; i++) {
      theta_data[b_tmp_data[i] - 1] = 6.2831853071795862;
    }
    loop_ub = theta_size[1];
    for (i = 0; i < loop_ub; i++) {
      k = theta_size[0];
      for (y_size_idx_0 = 0; y_size_idx_0 < k; y_size_idx_0++) {
        nx_tmp = y_size_idx_0 + theta_size[0] * i;
        theta_data[nx_tmp] -= 3.1415926535897931;
      }
    }
  }
}

/*
 * File trailer for wrapToPi.c
 *
 * [EOF]
 */
