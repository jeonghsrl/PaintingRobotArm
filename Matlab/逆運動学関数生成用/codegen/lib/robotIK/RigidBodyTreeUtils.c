/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: RigidBodyTreeUtils.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 27-Feb-2022 20:30:33
 */

/* Include Files */
#include "RigidBodyTreeUtils.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : const double config1[6]
 *                const double config2_data[]
 *                const int config2_size[2]
 *                double dist_data[]
 *                int *dist_size
 * Return Type  : void
 */
void RigidBodyTreeUtils_distance(const double config1[6],
                                 const double config2_data[],
                                 const int config2_size[2], double dist_data[],
                                 int *dist_size)
{
  double configDiff_data[192];
  double thetaWrap_data[192];
  double theta_data[192];
  double y_data[192];
  double b_y_data[32];
  double absxk;
  double scale;
  double t;
  double y;
  int configDiff_size_idx_0;
  int ibmat;
  int itilerow;
  int jcol;
  int k;
  int ntilerows;
  unsigned char b_tmp_data[192];
  bool pos_data[192];
  bool tmp_data[192];
  bool exitg1;
  bool varargout_1;
  if (1 > config2_size[0]) {
    configDiff_size_idx_0 = 1;
  } else if (1 < config2_size[0]) {
    ntilerows = config2_size[0];
    for (jcol = 0; jcol < 6; jcol++) {
      ibmat = jcol * ntilerows;
      for (itilerow = 0; itilerow < ntilerows; itilerow++) {
        configDiff_data[ibmat + itilerow] = config1[jcol];
      }
    }
    ntilerows = config2_size[0] * config2_size[1];
    configDiff_size_idx_0 = config2_size[0];
    for (ibmat = 0; ibmat < ntilerows; ibmat++) {
      configDiff_data[ibmat] = config2_data[ibmat] - configDiff_data[ibmat];
    }
  } else {
    configDiff_size_idx_0 = 1;
    for (ibmat = 0; ibmat < 6; ibmat++) {
      configDiff_data[ibmat] = config2_data[ibmat] - config1[ibmat];
    }
  }
  jcol = configDiff_size_idx_0;
  for (ibmat = 0; ibmat < 6; ibmat++) {
    for (ntilerows = 0; ntilerows < configDiff_size_idx_0; ntilerows++) {
      theta_data[ntilerows + configDiff_size_idx_0 * ibmat] =
          configDiff_data[ntilerows];
    }
  }
  ntilerows = configDiff_size_idx_0 * 6;
  for (k = 0; k < ntilerows; k++) {
    y_data[k] = fabs(configDiff_data[k % configDiff_size_idx_0]);
  }
  ntilerows = (signed char)configDiff_size_idx_0 * 6;
  for (ibmat = 0; ibmat < ntilerows; ibmat++) {
    pos_data[ibmat] = (y_data[ibmat] > 3.1415926535897931);
  }
  varargout_1 = false;
  ntilerows = 1;
  exitg1 = false;
  while ((!exitg1) && (ntilerows <= (signed char)configDiff_size_idx_0 * 6)) {
    if (!pos_data[ntilerows - 1]) {
      ntilerows++;
    } else {
      varargout_1 = true;
      exitg1 = true;
    }
  }
  if (varargout_1) {
    for (ibmat = 0; ibmat < 6; ibmat++) {
      for (ntilerows = 0; ntilerows < configDiff_size_idx_0; ntilerows++) {
        theta_data[ntilerows + configDiff_size_idx_0 * ibmat] =
            configDiff_data[ntilerows] + 3.1415926535897931;
      }
    }
    ntilerows = configDiff_size_idx_0 * 6;
    for (ibmat = 0; ibmat < ntilerows; ibmat++) {
      pos_data[ibmat] = (theta_data[ibmat] > 0.0);
    }
    ntilerows = (signed char)configDiff_size_idx_0 * 6;
    for (k = 0; k < ntilerows; k++) {
      thetaWrap_data[k] = b_mod(theta_data[k]);
    }
    for (ibmat = 0; ibmat < ntilerows; ibmat++) {
      tmp_data[ibmat] = (thetaWrap_data[ibmat] == 0.0);
    }
    ntilerows = (signed char)configDiff_size_idx_0 * 6 - 1;
    ibmat = 0;
    jcol = 0;
    for (itilerow = 0; itilerow <= ntilerows; itilerow++) {
      if (tmp_data[itilerow] && pos_data[itilerow]) {
        ibmat++;
        b_tmp_data[jcol] = (unsigned char)(itilerow + 1);
        jcol++;
      }
    }
    ntilerows = ibmat - 1;
    for (ibmat = 0; ibmat <= ntilerows; ibmat++) {
      thetaWrap_data[b_tmp_data[ibmat] - 1] = 6.2831853071795862;
    }
    jcol = (signed char)configDiff_size_idx_0;
    ntilerows = (signed char)configDiff_size_idx_0 * 6;
    for (ibmat = 0; ibmat < ntilerows; ibmat++) {
      theta_data[ibmat] = thetaWrap_data[ibmat] - 3.1415926535897931;
    }
  }
  for (ibmat = 0; ibmat < 6; ibmat++) {
    for (ntilerows = 0; ntilerows < jcol; ntilerows++) {
      configDiff_data[ntilerows] = theta_data[ntilerows + jcol * ibmat];
    }
  }
  for (ibmat = 0; ibmat < configDiff_size_idx_0; ibmat++) {
    for (ntilerows = 0; ntilerows < 6; ntilerows++) {
      theta_data[ntilerows + 6 * ibmat] =
          configDiff_data[ibmat + configDiff_size_idx_0 * ntilerows];
    }
  }
  jcol = (signed char)configDiff_size_idx_0;
  for (itilerow = 0; itilerow < configDiff_size_idx_0; itilerow++) {
    ntilerows = itilerow * 6;
    y = 0.0;
    scale = 3.3121686421112381E-170;
    ibmat = ntilerows + 6;
    for (k = ntilerows + 1; k <= ibmat; k++) {
      absxk = fabs(theta_data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }
    b_y_data[itilerow] = scale * sqrt(y);
  }
  *dist_size = (signed char)configDiff_size_idx_0;
  if (0 <= jcol - 1) {
    memcpy(&dist_data[0], &b_y_data[0], jcol * sizeof(double));
  }
}

/*
 * File trailer for RigidBodyTreeUtils.c
 *
 * [EOF]
 */
