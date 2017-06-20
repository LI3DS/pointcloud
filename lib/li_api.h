/***********************************************************************
* li_api.h
*
***********************************************************************/

#ifndef _LI_API_H
#define _LI_API_H

#include "li_api.h"

/* Linear Algebra types */
typedef double LIVEC3[3];
typedef double LIVEC4[4];
typedef double LIMAT33[9];
typedef double LIMAT43[12];
typedef double LIMAT44[16];
typedef LIVEC3 LIBOX3[2];
typedef LIVEC4 LIBOX4[2];

typedef struct
{
	LIMAT44 fwd;
	double *bwd;
	double det;
} LIFRUSTUM;

typedef struct
{
	double pps[2];
	double c[3];
	double r2max;
} LIDISTORSION;

/**********************************************************************
* PCPOINT
*/

/** rotate a point in place given a unit quaternion */
void li_point_rotate_quaternion(PCPOINT *point, double qw, double qx, double qy, double qz, const char *xdimname, const char *ydimname, const char *zdimname);

/** translate a point */
void li_point_translate(PCPOINT *point, double tx, double ty, double tz, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply an affine transformation to a point */
void li_point_affine(PCPOINT *point, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply a projective transformation to a point */
void li_point_projective(PCPOINT *point, double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double m, double n, double o, double p, const char *xdimname, const char *ydimname, const char *zdimname);

/**********************************************************************
* PCPATCH
*/

/** rotate a patch given a unit quaternion */
PCPATCH *li_patch_rotate_quaternion(const PCPATCH *patch, double qw, double qx, double qy, double qz, const char *xdimname, const char *ydimname, const char *zdimname);

/** translate a patch */
PCPATCH *li_patch_translate(const PCPATCH *patch, double tx, double ty, double tz, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply an affine transformation to a patch */
PCPATCH *li_patch_affine(const PCPATCH *patch, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply a projective transformation to a patch */
PCPATCH *li_patch_projective(const PCPATCH *patch, double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double m, double n, double o, double p, const char *xdimname, const char *ydimname, const char *zdimname);

/**********************************************************************
* LIBOX4
*/
LIBOX4 *li_box4d_affine(LIBOX4 ibox, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff);

LIBOX4 *li_box4d_rotate_quaternion(LIBOX4 ibox, double qw, double qx, double qy, double qz);

/** convert a box to a frustum (lossless) */
int li_frustum_from_box(LIFRUSTUM *res, const LIBOX3 b);

/** convert a frustum to a box (lossy if not axis-aligned) */
int  li_box_from_frustum(LIBOX3 res, const LIFRUSTUM *f);

/** convert a frustum to a postgis ewkb byte array */
uint8_t *li_frustum_to_geometry_wkb(const LIFRUSTUM *f, uint32_t srid, size_t *wkbsize);

/** test frustum intersection. valid modes are 1,2,3,4 */
int li_frustum_intersects(const LIFRUSTUM *f1, const LIFRUSTUM *f2, uint8_t mode);

/** test frustum containment */
int li_frustum_contains(const LIFRUSTUM *f1, const LIFRUSTUM *f2);

/** test frustum validity */
int li_frustum_is_valid(const LIFRUSTUM *f);

/** try to make the frustum valid */
int li_frustum_make_valid(LIFRUSTUM *res, const LIFRUSTUM *f);

/** 3D volume **/
double li_box_volume(const LIBOX3 b);

/** 3D volume, f should be valid **/
double li_frustum_volume(const LIFRUSTUM *f);

#endif /* _LI_API__H */
