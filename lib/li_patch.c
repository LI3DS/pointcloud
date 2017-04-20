/***********************************************************************
* li_patch.c
*
***********************************************************************/

#include <math.h>
#include <assert.h>
#include "pc_api_internal.h"
#include "li_api_internal.h"

/**
* Rotate a patch given a unit quaternion.
*/
PCPATCH *
li_patch_rotate_quaternion(
	const PCPATCH *patch,
	double qw, double qx, double qy, double qz,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	PCPATCH_UNCOMPRESSED *uncompressed_patch;
	PCDIMENSION *xdim, *ydim, *zdim;
	const PCSCHEMA *schema;
	PCPOINTLIST *pointlist;
	LIMAT33 qmat;
	LIVEC3 vec, rvec;
	size_t i;

	li_matrix_33_set_from_quaternion(qmat, qw, qx, qy, qz);

	if ( patch->type == PC_NONE )
	{
		pointlist = pc_pointlist_from_uncompressed((PCPATCH_UNCOMPRESSED *)patch);
		uncompressed_patch = pc_patch_uncompressed_from_pointlist(pointlist);
		pc_pointlist_free(pointlist);
	}
	else
	{
		uncompressed_patch = (PCPATCH_UNCOMPRESSED *)pc_patch_uncompress(patch);
	}
	if ( NULL == uncompressed_patch )
		return NULL;

	schema = uncompressed_patch->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->xdim);
		xdim = schema->xdim;
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->ydim);
		ydim = schema->ydim;
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->zdim )
	{
		zdim = schema->zdim;
	}
	else
	{
		zdim = NULL;
	}

	pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

	// the points of pointlist include pointers to the uncompressed patch data
	// block, so updating the points changes the patch payload

	for ( i = 0; i < pointlist->npoints; i++ )
	{
		PCPOINT *point = pc_pointlist_get_point(pointlist, i);

		pc_point_get_double(point, xdim, &vec[0]);
		pc_point_get_double(point, ydim, &vec[1]);
		if ( zdim )
			pc_point_get_double(point, zdim, &vec[2]);

		li_matrix_33_multiply_vector_3(rvec, qmat, vec);

		pc_point_set_double(point, xdim, rvec[0]);
		pc_point_set_double(point, ydim, rvec[1]);
		if ( zdim )
			pc_point_set_double(point, zdim, rvec[2]);
	}

	pc_pointlist_free(pointlist);

	if ( PC_FAILURE == pc_patch_uncompressed_compute_extent(uncompressed_patch) )
	{
		pcerror("%s: extent computation failed", __func__);
		return NULL;
	}
	if ( PC_FAILURE == pc_patch_uncompressed_compute_stats(uncompressed_patch) )
	{
		pcerror("%s: stats computation failed", __func__);
		return NULL;
	}


	return ((PCPATCH *)uncompressed_patch);
}

/**
* Translate a patch.
*/
PCPATCH *
li_patch_translate(
	const PCPATCH *patch,
	double tx, double ty, double tz,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	PCPATCH_UNCOMPRESSED *uncompressed_patch;
	PCDIMENSION *xdim, *ydim, *zdim;
	const PCSCHEMA *schema;
	PCPOINTLIST *pointlist;
	double x, y, z;
	size_t i;

	if ( patch->type == PC_NONE )
	{
		pointlist = pc_pointlist_from_uncompressed((PCPATCH_UNCOMPRESSED *)patch);
		uncompressed_patch = pc_patch_uncompressed_from_pointlist(pointlist);
		pc_pointlist_free(pointlist);
	}
	else
	{
		uncompressed_patch = (PCPATCH_UNCOMPRESSED *)pc_patch_uncompress(patch);
	}
	if ( NULL == uncompressed_patch )
		return NULL;

	schema = uncompressed_patch->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->xdim);
		xdim = schema->xdim;
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->ydim);
		ydim = schema->ydim;
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->zdim )
	{
		zdim = schema->zdim;
	}
	else
	{
		zdim = NULL;
	}

	pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

	// the points of pointlist include pointers to the uncompressed patch data
	// block, so updating the points changes the patch payload

	for ( i = 0; i < pointlist->npoints; i++ )
	{
		PCPOINT *point = pc_pointlist_get_point(pointlist, i);

		pc_point_get_double(point, xdim, &x);
		pc_point_get_double(point, ydim, &y);
		if ( zdim )
			pc_point_get_double(point, zdim, &z);

		pc_point_set_double(point, xdim, x + tx);
		pc_point_set_double(point, ydim, y + ty);
		if ( zdim )
			pc_point_set_double(point, zdim, z + tz);
	}

	pc_pointlist_free(pointlist);

	if ( PC_FAILURE == pc_patch_uncompressed_compute_extent(uncompressed_patch) )
	{
		pcerror("%s: extent computation failed", __func__);
		return NULL;
	}
	if ( PC_FAILURE == pc_patch_uncompressed_compute_stats(uncompressed_patch) )
	{
		pcerror("%s: stats computation failed", __func__);
		return NULL;
	}

	return ((PCPATCH *)uncompressed_patch);
}

/**
* Apply an affine transformation to a patch.
*/
PCPATCH *
li_patch_affine(
	const PCPATCH *patch,
	double a, double b, double c,
	double d, double e, double f,
	double g, double h, double i,
	double xoff, double yoff, double zoff,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	PCPATCH_UNCOMPRESSED *uncompressed_patch;
	PCDIMENSION *xdim, *ydim, *zdim;
	const PCSCHEMA *schema;
	PCPOINTLIST *pointlist;
	LIMAT43 amat;
	LIVEC3 vec, rvec;
	size_t idx;

	li_matrix_43_set(amat, a, b, c, xoff, d, e, f, yoff, g, h, i, zoff);

	if ( patch->type == PC_NONE )
	{
		pointlist = pc_pointlist_from_uncompressed((PCPATCH_UNCOMPRESSED *)patch);
		uncompressed_patch = pc_patch_uncompressed_from_pointlist(pointlist);
		pc_pointlist_free(pointlist);
	}
	else
	{
		uncompressed_patch = (PCPATCH_UNCOMPRESSED *)pc_patch_uncompress(patch);
	}
	if ( NULL == uncompressed_patch )
		return NULL;

	schema = uncompressed_patch->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->xdim);
		xdim = schema->xdim;
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->ydim);
		ydim = schema->ydim;
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->zdim )
	{
		zdim = schema->zdim;
	}
	else
	{
		zdim = NULL;
	}

	pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

	// the points of pointlist include pointers to the uncompressed patch data
	// block, so updating the points changes the patch payload

	for ( idx = 0; idx < pointlist->npoints; idx++ )
	{
		PCPOINT *point = pc_pointlist_get_point(pointlist, idx);

		pc_point_get_double(point, xdim, &vec[0]);
		pc_point_get_double(point, ydim, &vec[1]);
		if ( zdim )
			pc_point_get_double(point, zdim, &vec[2]);

		li_matrix_43_transform_affine(rvec, amat, vec);

		pc_point_set_double(point, xdim, rvec[0]);
		pc_point_set_double(point, ydim, rvec[1]);
		if ( zdim )
			pc_point_set_double(point, zdim, rvec[2]);
	}

	pc_pointlist_free(pointlist);

	if ( PC_FAILURE == pc_patch_uncompressed_compute_extent(uncompressed_patch) )
	{
		pcerror("%s: extent computation failed", __func__);
		return NULL;
	}
	if ( PC_FAILURE == pc_patch_uncompressed_compute_stats(uncompressed_patch) )
	{
		pcerror("%s: stats computation failed", __func__);
		return NULL;
	}

	return ((PCPATCH *)uncompressed_patch);
}

/**
* Apply a projective/perspective transformation to a patch.
*/
PCPATCH *
li_patch_projective(
	const PCPATCH *patch,
	double a, double b, double c, double d,
	double e, double f, double g, double h,
	double i, double j, double k, double l,
	double m, double n, double o, double p,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	PCPATCH_UNCOMPRESSED *uncompressed_patch;
	PCDIMENSION *xdim, *ydim, *zdim;
	const PCSCHEMA *schema;
	PCPOINTLIST *pointlist;
	LIMAT44 pmat;
	LIVEC3 vec, rvec;
	size_t idx;

	li_matrix_44_set(pmat, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p);

	if ( patch->type == PC_NONE )
	{
		pointlist = pc_pointlist_from_uncompressed((PCPATCH_UNCOMPRESSED *)patch);
		uncompressed_patch = pc_patch_uncompressed_from_pointlist(pointlist);
		pc_pointlist_free(pointlist);
	}
	else
	{
		uncompressed_patch = (PCPATCH_UNCOMPRESSED *)pc_patch_uncompress(patch);
	}
	if ( NULL == uncompressed_patch )
		return NULL;

	schema = uncompressed_patch->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->xdim);
		xdim = schema->xdim;
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->ydim);
		ydim = schema->ydim;
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->zdim )
	{
		zdim = schema->zdim;
	}
	else
	{
		zdim = NULL;
	}

	pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

	// the points of pointlist include pointers to the uncompressed patch data
	// block, so updating the points changes the patch payload

	for ( idx = 0; idx < pointlist->npoints; idx++ )
	{
		PCPOINT *point = pc_pointlist_get_point(pointlist, idx);

		pc_point_get_double(point, xdim, &vec[0]);
		pc_point_get_double(point, ydim, &vec[1]);
		if ( zdim )
			pc_point_get_double(point, zdim, &vec[2]);

		li_matrix_44_transform_projective_vector_3(rvec, pmat, vec);

		pc_point_set_double(point, xdim, rvec[0]);
		pc_point_set_double(point, ydim, rvec[1]);
		if ( zdim )
			pc_point_set_double(point, zdim, rvec[2]);
	}

	pc_pointlist_free(pointlist);

	if ( PC_FAILURE == pc_patch_uncompressed_compute_extent(uncompressed_patch) )
	{
		pcerror("%s: extent computation failed", __func__);
		return NULL;
	}
	if ( PC_FAILURE == pc_patch_uncompressed_compute_stats(uncompressed_patch) )
	{
		pcerror("%s: stats computation failed", __func__);
		return NULL;
	}

	return ((PCPATCH *)uncompressed_patch);
}