/***********************************************************************
* li_box4d.c
*
***********************************************************************/

#include "pc_api_internal.h"
#include "li_api_internal.h"
#include "stringbuffer.h"


typedef int (*transform_fn_t)(LIVEC3, const double *, const LIVEC3);


char *
li_box4d_as_box3d(LIBOX4 box)
{
	stringbuffer_t *sb = stringbuffer_create();
	char *str;

	stringbuffer_aprintf(sb, "BOX3D(%g %g %g,%g %g %g)",
						 box[0][0], box[0][1], box[0][2],
						 box[1][0], box[1][1], box[1][2]);
	str = stringbuffer_getstringcopy(sb);
	stringbuffer_destroy(sb);
	return str;
}


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

		if ( transform(rvec, mat, vec) == PC_FAILURE ) {
			pcfree(obox);
			return NULL;
		}

		for ( d = 0; d < 3; d++ )
		{
			if ( idx == 0 )
				(*obox)[0][d] = (*obox)[1][d] = rvec[d];
			else if ( rvec[d] < (*obox)[0][d] )
				(*obox)[0][d] = rvec[d];
			else if ( rvec[d] > (*obox)[1][d] )
				(*obox)[1][d] = rvec[d];
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
* Apply a projective pinhole transformation to a box4d.
*/
LIBOX4 *
li_box4d_projective_pinhole(LIBOX4 ibox, double focal, double px, double py)
{
	LIMAT44 pmat;

	li_matrix_44_set_from_pinhole_projection(pmat, px, py, focal, focal, 0);
	return li_box4d_transform(ibox, pmat, li_matrix_44_transform_projective_vector_3);
}
