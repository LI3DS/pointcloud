/***********************************************************************
* li_box4d.c
*
***********************************************************************/

#include "pc_api_internal.h"
#include "li_api_internal.h"

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
	int idx;
	LIMAT43 amat;
	LIVEC3 vec, rvec;
	LIBOX4 *obox;

	obox = pcalloc(sizeof(LIBOX4));

	// m values are unchanged
	(*obox)[0][3] = ibox[0][3];
	(*obox)[1][3] = ibox[1][3];

	li_matrix_43_set(amat, a, b, c, xoff, d, e, f, yoff, g, h, i, zoff);

	// transform each corner, and build the new box

	for ( idx = 0; idx < 8; idx++)
	{
		int d;

		vec[0] = ibox[(idx >> 0) % 2][0];
		vec[1] = ibox[(idx >> 1) % 2][1];
		vec[2] = ibox[(idx >> 2) % 2][2];

		li_matrix_43_transform_affine(rvec, amat, vec);

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
