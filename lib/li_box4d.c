/***********************************************************************
* li_box4d.c
*
***********************************************************************/

#include "pc_api_internal.h"
#include "li_api_internal.h"

#include <math.h>


typedef void (*transform_fn_t)(LIVEC3, const double *, const LIVEC3);


static LIBOX4 *
li_box4d_transform(LIBOX4 ibox, void *mat, transform_fn_t transform)
{
	int idx;
	LIBOX4 *obox;
	LIVEC3 vec, rvec;

	obox = pcalloc(sizeof(LIBOX4));

	// m values are unchanged
	(*obox)[0][3] = ibox[0][3];
	(*obox)[1][3] = ibox[1][3];

	// transform each corner, and build the new box

	for ( idx = 0; idx < 8; idx++)
	{
		int d;

		vec[0] = ibox[(idx >> 0) % 2][0];
		vec[1] = ibox[(idx >> 1) % 2][1];
		vec[2] = ibox[(idx >> 2) % 2][2];

		transform(rvec, mat, vec);

		if (idx == 0 )
			for ( d = 0; d < 3; d++ )
				(*obox)[0][d] = (*obox)[1][d] = rvec[d];
		else
			for ( d = 0; d < 3; d++ )
			{
				(*obox)[0][d] = fmin((*obox)[0][d], rvec[d]);
				(*obox)[1][d] = fmax((*obox)[1][d], rvec[d]);
			}

	}

	return obox;
}


/**
* Rotate a box4d given a unit quaternion.
*/
LIBOX4 *
li_box4d_rotate_quaternion(
	LIBOX4 ibox,
	double qw, double qx, double qy, double qz)
{
	LIMAT33 qmat;

	li_matrix_33_set_from_quaternion(qmat, qw, qx, qy, qz);
	return li_box4d_transform(ibox, qmat, li_matrix_33_multiply_vector_3);
}


/**
* Apply an affine transformation to a box4d.
*/
LIBOX4 *
li_box4d_affine(
	LIBOX4 ibox,
	double a, double b, double c,
	double d, double e, double f,
	double g, double h, double i,
	double xoff, double yoff, double zoff)
{
	LIMAT43 amat;

	li_matrix_43_set(amat, a, b, c, xoff, d, e, f, yoff, g, h, i, zoff);
	return li_box4d_transform(ibox, amat, li_matrix_43_transform_affine);
}


/**
* Undistort a box4d.
*/
LIBOX4 *
li_box4d_undistort(
	LIBOX4 ibox,
	double pps0, double pps1, double c0, double c1, double c2)
{
	int d;
	LIBOX3 res;
	LIDISTORSION dist;
	LIBOX4 *obox;
	const LIBOX3 box = {
		{ibox[0][0], ibox[0][1], ibox[0][2]},
		{ibox[1][0], ibox[1][1], ibox[1][2]}};

	li_distortion_set(&dist, pps0, pps1, c0, c1, c2);

	if ( li_box_transform_undistorsion(res, &dist, box) == PC_FAILURE )
		return NULL;

	obox = pcalloc(sizeof(LIBOX4));

	// m values are unchanged
	(*obox)[0][3] = ibox[0][3];
	(*obox)[1][3] = ibox[1][3];

	for ( d = 0; d < 3; d++)
	{
		(*obox)[0][d] = res[0][d];
		(*obox)[1][d] = res[1][d];
	}

	return obox;
}


/**
* Distort a box4d.
*/
LIBOX4 *
li_box4d_distort(
	LIBOX4 ibox,
	double pps0, double pps1, double c0, double c1, double c2)
{
	int d;
	LIBOX3 res;
	LIDISTORSION dist;
	LIBOX4 *obox;
	const LIBOX3 box = {
		{ibox[0][0], ibox[0][1], ibox[0][2]},
		{ibox[1][0], ibox[1][1], ibox[1][2]}};

	li_distortion_set(&dist, pps0, pps1, c0, c1, c2);

	if ( li_box_transform_distorsion(res, &dist, box) == PC_FAILURE )
		return NULL;

	obox = pcalloc(sizeof(LIBOX4));

	// m values are unchanged
	(*obox)[0][3] = ibox[0][3];
	(*obox)[1][3] = ibox[1][3];

	for ( d = 0; d < 3; d++)
	{
		(*obox)[0][d] = res[0][d];
		(*obox)[1][d] = res[1][d];
	}

	return obox;
}
