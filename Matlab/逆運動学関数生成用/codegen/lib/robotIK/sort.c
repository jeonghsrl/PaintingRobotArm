/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: sort.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "sort.h"
#include "rt_nonfinite.h"
#include "sortIdx.h"
#include "rt_nonfinite.h"

/* Type Definitions */
#ifndef struct_emxArray_int32_T_32
#define struct_emxArray_int32_T_32
struct emxArray_int32_T_32 {
  int data[32];
  int size[1];
};
#endif /* struct_emxArray_int32_T_32 */
#ifndef typedef_emxArray_int32_T_32
#define typedef_emxArray_int32_T_32
typedef struct emxArray_int32_T_32 emxArray_int32_T_32;
#endif /* typedef_emxArray_int32_T_32 */

/* Function Definitions */
/*
 * Arguments    : double x[4]
 * Return Type  : void
 */
void b_sort(double x[4])
{
  double tmp;
  double vwork_idx_0;
  double vwork_idx_1;
  vwork_idx_0 = x[0];
  vwork_idx_1 = x[2];
  if ((!(vwork_idx_0 <= vwork_idx_1)) && (!rtIsNaN(vwork_idx_1))) {
    tmp = vwork_idx_0;
    vwork_idx_0 = vwork_idx_1;
    vwork_idx_1 = tmp;
  }
  x[0] = vwork_idx_0;
  x[2] = vwork_idx_1;
  vwork_idx_0 = x[1];
  vwork_idx_1 = x[3];
  if ((!(vwork_idx_0 <= vwork_idx_1)) && (!rtIsNaN(vwork_idx_1))) {
    tmp = vwork_idx_0;
    vwork_idx_0 = vwork_idx_1;
    vwork_idx_1 = tmp;
  }
  x[1] = vwork_idx_0;
  x[3] = vwork_idx_1;
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 * Return Type  : void
 */
void c_sort(double x_data[], const int *x_size)
{
  emxArray_int32_T_32 b_vwork_data;
  double vwork_data[32];
  int dim;
  int k;
  int vlen;
  int vstride;
  int vwork_size;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (dim = 0; dim < vstride; dim++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[dim + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, b_vwork_data.data, &b_vwork_data.size[0]);
    for (k = 0; k <= vlen; k++) {
      x_data[dim + k * vstride] = vwork_data[k];
    }
  }
}

/*
 * Arguments    : double x_data[]
 *                const int *x_size
 *                int idx_data[]
 *                int *idx_size
 * Return Type  : void
 */
void d_sort(double x_data[], const int *x_size, int idx_data[], int *idx_size)
{
  double vwork_data[32];
  int iidx_data[32];
  int dim;
  int j;
  int k;
  int vlen;
  int vstride;
  int vwork_size;
  dim = 0;
  if (*x_size != 1) {
    dim = -1;
  }
  if (dim + 2 <= 1) {
    vwork_size = *x_size;
  } else {
    vwork_size = 1;
  }
  vlen = vwork_size - 1;
  *idx_size = *x_size;
  vstride = 1;
  for (k = 0; k <= dim; k++) {
    vstride *= *x_size;
  }
  for (j = 0; j < vstride; j++) {
    for (k = 0; k <= vlen; k++) {
      vwork_data[k] = x_data[j + k * vstride];
    }
    sortIdx(vwork_data, &vwork_size, iidx_data, &dim);
    for (k = 0; k <= vlen; k++) {
      dim = j + k * vstride;
      x_data[dim] = vwork_data[k];
      idx_data[dim] = iidx_data[k];
    }
  }
}

/*
 * Arguments    : double x[2]
 * Return Type  : void
 */
void sort(double x[2])
{
  double tmp;
  if ((!(x[0] <= x[1])) && (!rtIsNaN(x[1]))) {
    tmp = x[0];
    x[0] = x[1];
    x[1] = tmp;
  }
}

/*
 * File trailer for sort.c
 *
 * [EOF]
 */
