/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: all.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "all.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const bool x_data[]
 *                const int x_size[2]
 *                bool y_data[]
 *                int *y_size
 * Return Type  : void
 */
void all(const bool x_data[], const int x_size[2], bool y_data[], int *y_size)
{
  int i1;
  int i2;
  int ix;
  int j;
  int vstride;
  bool exitg1;
  vstride = (signed char)x_size[0];
  *y_size = (signed char)x_size[0];
  for (i2 = 0; i2 < vstride; i2++) {
    y_data[i2] = true;
  }
  vstride = x_size[0];
  i2 = (x_size[1] - 1) * x_size[0];
  i1 = 0;
  for (j = 0; j < vstride; j++) {
    i1++;
    i2++;
    ix = i1;
    exitg1 = false;
    while ((!exitg1) && ((vstride > 0) && (ix <= i2))) {
      if (!x_data[ix - 1]) {
        y_data[j] = false;
        exitg1 = true;
      } else {
        ix += vstride;
      }
    }
  }
}

/*
 * File trailer for all.c
 *
 * [EOF]
 */
