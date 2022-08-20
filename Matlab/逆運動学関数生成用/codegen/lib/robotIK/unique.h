/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: unique.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

#ifndef UNIQUE_H
#define UNIQUE_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void unique_rows(const double a_data[], const int a_size[2], double b_data[],
                 int b_size[2], int ndx_data[], int *ndx_size);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for unique.h
 *
 * [EOF]
 */
