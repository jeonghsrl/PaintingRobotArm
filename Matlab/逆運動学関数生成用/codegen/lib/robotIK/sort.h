/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

#ifndef SORT_H
#define SORT_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_sort(double x[4]);

void c_sort(double x_data[], const int *x_size);

void d_sort(double x_data[], const int *x_size, int idx_data[], int *idx_size);

void sort(double x[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for sort.h
 *
 * [EOF]
 */
