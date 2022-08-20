/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: robotIK.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

#ifndef ROBOTIK_H
#define ROBOTIK_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void robotIK(const double eeTform[16], unsigned char enforceJointLimits,
                    unsigned char sortByDistance,
                    const double referenceConfig[6], double qOpts_data[],
                    int qOpts_size[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for robotIK.h
 *
 * [EOF]
 */
