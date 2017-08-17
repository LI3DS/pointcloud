/***********************************************************************
* li_api_internal.h
*
***********************************************************************/

#ifndef _LI_API_INTERNAL_H
#define _LI_API_INTERNAL_H

#include "li_api.h"

/****************************************************************************
* MATRIX
*/

void li_matrix_43_set(LIMAT43 mat, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff);
void li_matrix_44_set(LIMAT44 mat, double a,  double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double m, double n, double o, double p);
void li_matrix_33_set_from_quaternion(LIMAT33 mat, double qw, double qx, double qy, double qz);
void li_matrix_33_multiply_vector_3(LIVEC3 rotatedvec, const LIMAT33 mat, const LIVEC3 vec);
void li_matrix_43_transform_affine(LIVEC3 res, const LIMAT43 mat, const LIVEC3 vec);
int li_matrix_44_transform_projective_vector_3(LIVEC3 res, const LIMAT44 mat, const LIVEC3 vec);
void li_matrix_44_transpose(LIMAT44 res, const LIMAT44 mat);
void li_matrix_44_multiply_matrix_44(LIMAT44 res, const LIMAT44 mat1, const LIMAT44 mat2);
void li_matrix_44_multiply_vector_3(LIVEC4 res, const LIMAT44 mat, const LIVEC3 vec);
void li_matrix_44_multiply_vector_4(LIVEC4 res, const LIMAT44 mat, const LIVEC4 vec);
double li_matrix_44_determinant(const LIMAT44 m);
void li_matrix_44_adjugate(LIMAT44 res, const LIMAT44 m, double *determinant);
int li_matrix_44_inverse(LIMAT44 res, const LIMAT44 m);
void li_distortion_set(LIDISTORSION*, double, double, double, double, double);
int li_box_transform_distorsion(LIBOX3, const LIDISTORSION*, const LIBOX3);
int li_box_transform_undistorsion(LIBOX3, const LIDISTORSION*, const LIBOX3);

void li_matrix_44_warn(const char *func, int line, const char *name, const LIMAT44 m);
void li_vector_3_warn(const char *func, int line, const char *name, const LIVEC4 v);
void li_vector_4_warn(const char *func, int line, const char *name, const LIVEC4 v);

#define LIWARND(x) pcwarn("%s:%d %s=%d",__func__,__LINE__,#x,x);
#define LIWARNLF(x) pcwarn("%s:%d %s=%lf",__func__,__LINE__,#x,x);
#define LIWARNVEC3(x) li_vector_3_warn(__func__,__LINE__,#x,x);
#define LIWARNVEC4(x) li_vector_4_warn(__func__,__LINE__,#x,x);
#define LIWARNMAT44(x) li_matrix_44_warn(__func__,__LINE__,#x,x);

#endif /* _LI_API_INTERNAL_H */
