/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rotm2eul.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "rotm2eul.h"
#include "robotIK_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const double R[9]
 *                double eul[3]
 * Return Type  : void
 */
void rotm2eul(const double R[9], double eul[3])
{
  double eul_tmp;
  double sy;
  sy = sqrt(R[5] * R[5] + R[2] * R[2]);
  eul[0] = rt_atan2d_snf(R[5], R[2]);
  eul_tmp = rt_atan2d_snf(sy, R[8]);
  eul[1] = eul_tmp;
  eul[2] = rt_atan2d_snf(R[7], -R[6]);
  if (sy < 2.2204460492503131E-15) {
    sy = -R[1];
    sy = rt_atan2d_snf(sy, R[4]);
    eul[1] = eul_tmp;
    eul[0] = sy;
    eul[2] = 0.0;
  }
  eul[0] = -eul[0];
  eul[1] = -eul[1];
  eul[2] = -eul[2];
  sy = eul[0];
  eul[0] = eul[2];
  eul[2] = sy;
}

/*
 * File trailer for rotm2eul.c
 *
 * [EOF]
 */
