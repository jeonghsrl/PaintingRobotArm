/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: unique.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "unique.h"
#include "rt_nonfinite.h"
#include "sortLE.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double a_data[]
 *                const int a_size[2]
 *                double b_data[]
 *                int b_size[2]
 *                int ndx_data[]
 *                int *ndx_size
 * Return Type  : void
 */
void unique_rows(const double a_data[], const int a_size[2], double b_data[],
                 int b_size[2], int ndx_data[], int *ndx_size)
{
  double ycol_data[32];
  double a;
  double absx;
  double b;
  int idx_data[32];
  int iwork_data[32];
  int col_data[6];
  int col_size[2];
  int b_i;
  int exitg1;
  int exponent;
  int i;
  int i1;
  int i2;
  int idx_size;
  int j;
  int k;
  int kEnd;
  int n;
  int p;
  int pEnd;
  int q;
  int qEnd;
  bool b_p;
  bool exitg2;
  if (a_size[0] == 0) {
    b_size[0] = 0;
    b_size[1] = a_size[1];
    *ndx_size = 0;
  } else {
    b_size[0] = a_size[0];
    b_size[1] = a_size[1];
    pEnd = a_size[0] * a_size[1];
    if (0 <= pEnd - 1) {
      memcpy(&b_data[0], &a_data[0], pEnd * sizeof(double));
    }
    n = a_size[1];
    col_size[0] = 1;
    col_size[1] = a_size[1];
    for (k = 0; k < n; k++) {
      col_data[k] = k + 1;
    }
    n = a_size[0] + 1;
    idx_size = a_size[0];
    pEnd = a_size[0];
    if (0 <= pEnd - 1) {
      memset(&idx_data[0], 0, pEnd * sizeof(int));
    }
    if (a_size[1] == 0) {
      for (k = 0; k <= n - 2; k++) {
        idx_data[k] = k + 1;
      }
    } else {
      i = a_size[0] - 1;
      for (k = 1; k <= i; k += 2) {
        if (sortLE(a_data, a_size, col_data, col_size, k, k + 1)) {
          idx_data[k - 1] = k;
          idx_data[k] = k + 1;
        } else {
          idx_data[k - 1] = k + 1;
          idx_data[k] = k;
        }
      }
      if ((a_size[0] & 1) != 0) {
        idx_data[a_size[0] - 1] = a_size[0];
      }
      b_i = 2;
      while (b_i < n - 1) {
        i2 = b_i << 1;
        j = 1;
        for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
          p = j;
          q = pEnd;
          qEnd = j + i2;
          if (qEnd > n) {
            qEnd = n;
          }
          k = 0;
          kEnd = qEnd - j;
          while (k + 1 <= kEnd) {
            i = idx_data[q - 1];
            i1 = idx_data[p - 1];
            if (sortLE(a_data, a_size, col_data, col_size, i1, i)) {
              iwork_data[k] = i1;
              p++;
              if (p == pEnd) {
                while (q < qEnd) {
                  k++;
                  iwork_data[k] = idx_data[q - 1];
                  q++;
                }
              }
            } else {
              iwork_data[k] = i;
              q++;
              if (q == qEnd) {
                while (p < pEnd) {
                  k++;
                  iwork_data[k] = idx_data[p - 1];
                  p++;
                }
              }
            }
            k++;
          }
          for (k = 0; k < kEnd; k++) {
            idx_data[(j + k) - 1] = iwork_data[k];
          }
          j = qEnd;
        }
        b_i = i2;
      }
    }
    i2 = a_size[0] - 1;
    n = a_size[1];
    for (j = 0; j < n; j++) {
      for (b_i = 0; b_i <= i2; b_i++) {
        ycol_data[b_i] = b_data[(idx_data[b_i] + b_size[0] * j) - 1];
      }
      for (b_i = 0; b_i <= i2; b_i++) {
        b_data[b_i + b_size[0] * j] = ycol_data[b_i];
      }
    }
    for (i = 0; i < idx_size; i++) {
      ycol_data[i] = idx_data[i];
    }
    q = -1;
    pEnd = a_size[0];
    k = 0;
    while (k + 1 <= pEnd) {
      p = k;
      do {
        exitg1 = 0;
        k++;
        if (k + 1 > pEnd) {
          exitg1 = 1;
        } else {
          b_p = false;
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j <= b_size[1] - 1)) {
            i2 = b_size[0] * j;
            a = b_data[p + i2];
            b = b_data[k + i2];
            absx = fabs(b / 2.0);
            if ((!rtIsInf(absx)) && (!rtIsNaN(absx))) {
              if (absx <= 2.2250738585072014E-308) {
                absx = 4.94065645841247E-324;
              } else {
                frexp(absx, &exponent);
                absx = ldexp(1.0, exponent - 53);
              }
            } else {
              absx = rtNaN;
            }
            if ((fabs(b - a) < absx) ||
                (rtIsInf(a) && rtIsInf(b) && ((a > 0.0) == (b > 0.0)))) {
              j++;
            } else {
              b_p = true;
              exitg2 = true;
            }
          }
          if (b_p) {
            exitg1 = 1;
          }
        }
      } while (exitg1 == 0);
      q++;
      i = b_size[1];
      for (j = 0; j < i; j++) {
        i2 = b_size[0] * j;
        b_data[q + i2] = b_data[p + i2];
      }
      ycol_data[q] = ycol_data[p];
    }
    if (1 > q + 1) {
      pEnd = 0;
    } else {
      pEnd = q + 1;
    }
    i2 = a_size[1] - 1;
    for (i = 0; i <= i2; i++) {
      for (i1 = 0; i1 < pEnd; i1++) {
        b_data[i1 + pEnd * i] = b_data[i1 + b_size[0] * i];
      }
    }
    b_size[0] = pEnd;
    b_size[1] = a_size[1];
    *ndx_size = q + 1;
    for (k = 0; k <= q; k++) {
      ndx_data[k] = (int)ycol_data[k];
    }
  }
}

/*
 * File trailer for unique.c
 *
 * [EOF]
 */
