/***********************************************************************
* li_point.c
*
***********************************************************************/

#include <assert.h>
#include "pc_api_internal.h"
#include "li_api_internal.h"

/**
* Rotate a point in place given a unit quaternion.
*/
void
li_point_rotate_quaternion(
	PCPOINT *point,
	double qw, double qx, double qy, double qz,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	const PCSCHEMA *schema;
	const PCDIMENSION *xdim, *ydim, *zdim;
	LIMAT33 qmat;
	LIVEC3 vec, rvec;

	li_matrix_33_set_from_quaternion(qmat, qw, qx, qy, qz);

	schema = point->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->x_position >= 0);
		xdim = pc_schema_get_dimension(schema, schema->x_position);
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->y_position >= 0);
		ydim = pc_schema_get_dimension(schema, schema->y_position);
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->z_position >= 0 )
	{
		zdim = pc_schema_get_dimension(schema, schema->z_position);
	}
	else
	{
		zdim = NULL;
	}

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

/**
* Translate a point.
*/
void
li_point_translate(
	PCPOINT *point,
	double tx, double ty, double tz,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	const PCSCHEMA *schema;
	const PCDIMENSION *xdim, *ydim, *zdim;
	double x, y, z;

	schema = point->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->x_position >= 0);
		xdim = pc_schema_get_dimension(schema, schema->x_position);
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->y_position >= 0);
		ydim = pc_schema_get_dimension(schema, schema->y_position);
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->z_position >= 0 )
	{
		zdim = pc_schema_get_dimension(schema, schema->z_position);
	}
	else
	{
		zdim = NULL;
	}

	pc_point_get_double(point, xdim, &x);
	pc_point_get_double(point, ydim, &y);
	if ( zdim )
		pc_point_get_double(point, zdim, &z);

	pc_point_set_double(point, xdim, x + tx);
	pc_point_set_double(point, ydim, y + ty);
	if ( zdim )
		pc_point_set_double(point, zdim, z + tz);
}

/**
* Apply an affine transformation to a point.
*/
void
li_point_affine(
	PCPOINT *point,
	double a, double b, double c,
	double d, double e, double f,
	double g, double h, double i,
	double xoff, double yoff, double zoff,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	const PCSCHEMA *schema;
	const PCDIMENSION *xdim, *ydim, *zdim;
	LIMAT43 amat;
	LIVEC3 vec, rvec;

	li_matrix_43_set(amat, a, b, c, xoff, d, e, f, yoff, g, h, i, zoff);

	schema = point->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->x_position >= 0);
		xdim = pc_schema_get_dimension(schema, schema->x_position);
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->y_position >= 0);
		ydim = pc_schema_get_dimension(schema, schema->y_position);
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->z_position >= 0 )
	{
		zdim = pc_schema_get_dimension(schema, schema->z_position);
	}
	else
	{
		zdim = NULL;
	}

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

/**
* Apply a projective/perspective transformation to a point.
*/
void
li_point_projective(
	PCPOINT *point,
	double a, double b, double c, double d,
	double e, double f, double g, double h,
	double i, double j, double k, double l,
	double m, double n, double o, double p,
	const char *xdimname, const char *ydimname, const char *zdimname)
{
	const PCSCHEMA *schema;
	const PCDIMENSION *xdim, *ydim, *zdim;
	LIMAT44 amat;
	LIVEC3 vec, rvec;

	li_matrix_44_set(amat, a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p);

	schema = point->schema;

	if ( xdimname )
	{
		xdim = pc_schema_get_dimension_by_name(schema, xdimname);
	}
	else
	{
		assert(schema->x_position >= 0);
		xdim = pc_schema_get_dimension(schema, schema->x_position);
	}
	if ( ydimname )
	{
		ydim = pc_schema_get_dimension_by_name(schema, ydimname);
	}
	else
	{
		assert(schema->y_position >= 0);
		ydim = pc_schema_get_dimension(schema, schema->y_position);
	}
	if ( zdimname )
	{
		zdim = pc_schema_get_dimension_by_name(schema, zdimname);
	}
	else if ( schema->z_position >= 0 )
	{
		zdim = pc_schema_get_dimension(schema, schema->z_position);
	}
	else
	{
		zdim = NULL;
	}

	pc_point_get_double(point, xdim, &vec[0]);
	pc_point_get_double(point, ydim, &vec[1]);
	if ( zdim )
		pc_point_get_double(point, zdim, &vec[2]);

	li_matrix_44_transform_projective_vector_3(rvec, amat, vec);

	pc_point_set_double(point, xdim, rvec[0]);
	pc_point_set_double(point, ydim, rvec[1]);
	if ( zdim )
		pc_point_set_double(point, zdim, rvec[2]);
}
