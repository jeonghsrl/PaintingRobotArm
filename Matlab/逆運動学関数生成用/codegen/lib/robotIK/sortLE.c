/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sortLE.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "sortLE.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double v_data[]
 *                const int v_size[2]
 *                const int dir_data[]
 *                const int dir_size[2]
 *                int idx1
 *                int idx2
 * Return Type  : bool
 */
bool sortLE(const double v_data[], const int v_size[2], const int dir_data[],
            const int dir_size[2], int idx1, int idx2)
{
  double v1;
  double v2;
  int k;
  int v1_tmp;
  bool exitg1;
  bool p;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= dir_size[1] - 1)) {
    v1_tmp = v_size[0] * (dir_data[k] - 1);
    v1 = v_data[(idx1 + v1_tmp) - 1];
    v2 = v_data[(idx2 + v1_tmp) - 1];
    if ((v1 == v2) || (rtIsNaN(v1) && rtIsNaN(v2))) {
      k++;
    } else {
      if ((!(v1 <= v2)) && (!rtIsNaN(v2))) {
        p = false;
      }
      exitg1 = true;
    }
  }
  return p;
}

/*
 * File trailer for sortLE.c
 *
 * [EOF]
 */
