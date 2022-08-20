/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_robotIK_api.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

#ifndef _CODER_ROBOTIK_API_H
#define _CODER_ROBOTIK_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void robotIK(real_T eeTform[16], uint8_T enforceJointLimits,
             uint8_T sortByDistance, real_T referenceConfig[6],
             real_T qOpts_data[], int32_T qOpts_size[2]);

void robotIK_api(const mxArray *const prhs[4], const mxArray **plhs);

void robotIK_atexit(void);

void robotIK_initialize(void);

void robotIK_terminate(void);

void robotIK_xil_shutdown(void);

void robotIK_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_robotIK_api.h
 *
 * [EOF]
 */
