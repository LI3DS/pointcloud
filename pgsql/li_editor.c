/***********************************************************************
* li_editor.c
*
*  Editor functions for points and patches in PgSQL.
*
*  PgSQL Pointcloud is free and open source software provided
*  by the Government of Canada
*
*  Copyright (c) 2013 Natural Resources Canada
*  Copyright (c) 2017 Oslandia
*
***********************************************************************/

#include "pc_pgsql.h"	   /* Common PgSQL support for our type */

Datum lipatch_rotate_quaternion(PG_FUNCTION_ARGS);
Datum lipatch_translate(PG_FUNCTION_ARGS);
Datum lipatch_affine(PG_FUNCTION_ARGS);
Datum lipatch_projective(PG_FUNCTION_ARGS);
Datum lipoint_rotate_quaternion(PG_FUNCTION_ARGS);
Datum lipoint_translate(PG_FUNCTION_ARGS);
Datum lipoint_affine(PG_FUNCTION_ARGS);
Datum lipoint_projective(PG_FUNCTION_ARGS);

static float8* li_getarg_float8_array(FunctionCallInfoData *fcinfo, int pos, int num_elts)
{
	ArrayType *array = PG_GETARG_ARRAYTYPE_P(pos);

	if ( ARR_ELEMTYPE(array) != FLOAT8OID ) {
		elog(ERROR, "array must be of type float8[]");
		return NULL;
	}
	if ( ARR_NDIM(array) != 1 ) {
		elog(ERROR, "array must have only one dimension");
		return NULL;
	}
	if ( ARR_HASNULL(array) ) {
		elog(ERROR, "array must not have null elements");
		return NULL;
	}
	if ( ARR_DIMS(array)[0] != num_elts ) {
		elog(ERROR, "array must have %d elements", num_elts);
		return NULL;
	}

	return (float8 *) ARR_DATA_PTR(array);
}

/**
* Rotate a patch based on a rotation quaternion
* PC_RotateQuaternion(patch pcpatch, qw float8, qx float8, qy float8, qz float8,
*					  xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(lipatch_rotate_quaternion);
Datum lipatch_rotate_quaternion(PG_FUNCTION_ARGS)
{
	SERIALIZED_PATCH *serpatch;
	PCPATCH *patch_in, *patch_out;
	PCSCHEMA *schema;
	float8 qw, qx, qy, qz;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpatch = PG_GETARG_SERPATCH_P(0);
	qw = PG_GETARG_FLOAT8(1);
	qx = PG_GETARG_FLOAT8(2);
	qy = PG_GETARG_FLOAT8(3);
	qz = PG_GETARG_FLOAT8(4);
	xdimname = PG_ARGISNULL(5) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(5));
	ydimname = PG_ARGISNULL(6) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(6));
	zdimname = PG_ARGISNULL(7) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(7));

	schema = pc_schema_from_pcid(serpatch->pcid, fcinfo);

	patch_in = pc_patch_deserialize(serpatch, schema);
	if ( ! patch_in )
	{
		elog(ERROR, "failed to deserialize patch");
		PG_RETURN_NULL();
	}

	patch_out = li_patch_rotate_quaternion(
		patch_in, qw, qx, qy, qz, xdimname, ydimname, zdimname);
	if ( ! patch_out )
	{
		elog(ERROR, "failed to rotate patch");
		PG_RETURN_NULL();
	}

	serpatch = pc_patch_serialize(patch_out, NULL);

	pc_patch_free(patch_in);
	pc_patch_free(patch_out);

	PG_RETURN_POINTER(serpatch);
}

/**
* Translate a patch
* PC_Translate(patch pcpatch, tx float8, ty float8, tz float8,
*			   xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(lipatch_translate);
Datum lipatch_translate(PG_FUNCTION_ARGS)
{
	SERIALIZED_PATCH *serpatch;
	PCPATCH *patch_in, *patch_out;
	PCSCHEMA *schema;
	float8 tx, ty, tz;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpatch = PG_GETARG_SERPATCH_P(0);
	tx = PG_GETARG_FLOAT8(1);
	ty = PG_GETARG_FLOAT8(2);
	tz = PG_GETARG_FLOAT8(3);
	xdimname = PG_ARGISNULL(4) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(4));
	ydimname = PG_ARGISNULL(5) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(5));
	zdimname = PG_ARGISNULL(6) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(6));

	schema = pc_schema_from_pcid(serpatch->pcid, fcinfo);

	patch_in = pc_patch_deserialize(serpatch, schema);
	if ( ! patch_in )
	{
		elog(ERROR, "failed to deserialize patch");
		PG_RETURN_NULL();
	}

	patch_out = li_patch_translate(
		patch_in, tx, ty, tz, xdimname, ydimname, zdimname);
	if ( ! patch_out )
	{
		elog(ERROR, "failed to rotate patch");
		PG_RETURN_NULL();
	}

	serpatch = pc_patch_serialize(patch_out, NULL);

	pc_patch_free(patch_in);
	pc_patch_free(patch_out);

	PG_RETURN_POINTER(serpatch);
}

/**
* Apply an affine transformation to a patch
* PC_Affine(patch pcpatch,
*			a float8, b float8, c float8,
*			d float8, e float8, f float8,
*			g float8, h float8, i float8,
*			xoff float8, yoff float8, zoff float8,
*			xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(lipatch_affine);
Datum lipatch_affine(PG_FUNCTION_ARGS)
{
	SERIALIZED_PATCH *serpatch;
	PCPATCH *patch_in, *patch_out;
	PCSCHEMA *schema;
	float8 a, b, c, d, e, f, g, h, i;
	float8 xoff, yoff, zoff;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpatch = PG_GETARG_SERPATCH_P(0);
	a = PG_GETARG_FLOAT8(1);
	b = PG_GETARG_FLOAT8(2);
	c = PG_GETARG_FLOAT8(3);
	d = PG_GETARG_FLOAT8(4);
	e = PG_GETARG_FLOAT8(5);
	f = PG_GETARG_FLOAT8(6);
	g = PG_GETARG_FLOAT8(7);
	h = PG_GETARG_FLOAT8(8);
	i = PG_GETARG_FLOAT8(9);
	xoff = PG_GETARG_FLOAT8(10);
	yoff = PG_GETARG_FLOAT8(11);
	zoff = PG_GETARG_FLOAT8(12);
	xdimname = PG_ARGISNULL(13) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(13));
	ydimname = PG_ARGISNULL(14) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(14));
	zdimname = PG_ARGISNULL(15) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(15));

	schema = pc_schema_from_pcid(serpatch->pcid, fcinfo);

	patch_in = pc_patch_deserialize(serpatch, schema);
	if ( ! patch_in )
	{
		elog(ERROR, "failed to deserialize patch");
		PG_RETURN_NULL();
	}

	patch_out = li_patch_affine(patch_in,
		a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
		xdimname, ydimname, zdimname);
	if ( ! patch_out )
	{
		elog(ERROR, "failed to rotate patch");
		PG_RETURN_NULL();
	}

	serpatch = pc_patch_serialize(patch_out, NULL);

	pc_patch_free(patch_in);
	pc_patch_free(patch_out);

	PG_RETURN_POINTER(serpatch);
}

/**
* Apply an projective transformation to a patch
* PC_Projective(patch pcpatch,
*			a float8, b float8, c float8, d float8,
*			e float8, f float8, g float8, h float8,
*			i float8, j float8, k float8, l float8,
*			m float8, n float8, o float8, p float8,
*			xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(lipatch_projective);
Datum lipatch_projective(PG_FUNCTION_ARGS)
{
	SERIALIZED_PATCH *serpatch;
	PCPATCH *patch_in, *patch_out;
	PCSCHEMA *schema;
	float8 a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpatch = PG_GETARG_SERPATCH_P(0);
	a = PG_GETARG_FLOAT8(1);
	b = PG_GETARG_FLOAT8(2);
	c = PG_GETARG_FLOAT8(3);
	d = PG_GETARG_FLOAT8(4);
	e = PG_GETARG_FLOAT8(5);
	f = PG_GETARG_FLOAT8(6);
	g = PG_GETARG_FLOAT8(7);
	h = PG_GETARG_FLOAT8(8);
	i = PG_GETARG_FLOAT8(9);
	j = PG_GETARG_FLOAT8(10);
	k = PG_GETARG_FLOAT8(11);
	l = PG_GETARG_FLOAT8(12);
	m = PG_GETARG_FLOAT8(13);
	n = PG_GETARG_FLOAT8(14);
	o = PG_GETARG_FLOAT8(15);
	p = PG_GETARG_FLOAT8(16);
	xdimname = text_to_cstring(PG_GETARG_TEXT_P(17));
	ydimname = text_to_cstring(PG_GETARG_TEXT_P(18));
	zdimname = text_to_cstring(PG_GETARG_TEXT_P(19));

	schema = pc_schema_from_pcid(serpatch->pcid, fcinfo);

	patch_in = pc_patch_deserialize(serpatch, schema);
	if ( ! patch_in )
	{
		elog(ERROR, "failed to deserialize patch");
		PG_RETURN_NULL();
	}

	patch_out = li_patch_projective(patch_in,
		a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p,
		xdimname, ydimname, zdimname);
	if ( ! patch_out )
	{
		elog(ERROR, "failed to project patch");
		PG_RETURN_NULL();
	}

	serpatch = pc_patch_serialize(patch_out, NULL);

	pc_patch_free(patch_in);
	pc_patch_free(patch_out);

	PG_RETURN_POINTER(serpatch);
}

/**
* Rotate a point based on a rotation quaternion
* PC_RotateQuaternion(point pcpoint, qw float8, qx float8, qy float8, qz float8,
*					  xdimname text, ydimname text, zdimname text) returns pcpoint
*/
PG_FUNCTION_INFO_V1(lipoint_rotate_quaternion);
Datum lipoint_rotate_quaternion(PG_FUNCTION_ARGS)
{
	SERIALIZED_POINT *serpoint;
	PCPOINT *point;
	PCSCHEMA *schema;
	float8 qw, qx, qy, qz;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpoint = PG_GETARG_SERPOINT_P(0);
	qw = PG_GETARG_FLOAT8(1);
	qx = PG_GETARG_FLOAT8(2);
	qy = PG_GETARG_FLOAT8(3);
	qz = PG_GETARG_FLOAT8(4);
	xdimname = PG_ARGISNULL(5) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(5));
	ydimname = PG_ARGISNULL(6) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(6));
	zdimname = PG_ARGISNULL(7) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(7));

	schema = pc_schema_from_pcid(serpoint->pcid, fcinfo);

	point = pc_point_deserialize(serpoint, schema);
	if ( ! point )
	{
		elog(ERROR, "failed to deserialize point");
		PG_RETURN_NULL();
	}

	li_point_rotate_quaternion(
		point, qw, qx, qy, qz, xdimname, ydimname, zdimname);

	serpoint = pc_point_serialize(point);

	pc_point_free(point);

	PG_RETURN_POINTER(serpoint);
}

/**
* Translate a point
* PC_Translate(point pcpoint, tx float8, ty float8, tz float8,
*			   xdimname text, ydimname text, zdimname text) returns pcpoint
*/
PG_FUNCTION_INFO_V1(lipoint_translate);
Datum lipoint_translate(PG_FUNCTION_ARGS)
{
	SERIALIZED_POINT *serpoint;
	PCPOINT *point;
	PCSCHEMA *schema;
	float8 tx, ty, tz;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpoint = PG_GETARG_SERPOINT_P(0);
	tx = PG_GETARG_FLOAT8(1);
	ty = PG_GETARG_FLOAT8(2);
	tz = PG_GETARG_FLOAT8(3);
	xdimname = PG_ARGISNULL(4) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(4));
	ydimname = PG_ARGISNULL(5) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(5));
	zdimname = PG_ARGISNULL(6) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(6));

	schema = pc_schema_from_pcid(serpoint->pcid, fcinfo);

	point = pc_point_deserialize(serpoint, schema);
	if ( ! point )
	{
		elog(ERROR, "failed to deserialize point");
		PG_RETURN_NULL();
	}

	li_point_translate(
		point, tx, ty, tz, xdimname, ydimname, zdimname);

	serpoint = pc_point_serialize(point);

	pc_point_free(point);

	PG_RETURN_POINTER(serpoint);
}

/**
* Apply an affine transformation to a point
* PC_Affine(point pcpoint,
*			a float8, b float8, c float8,
*			d float8, e float8, f float8,
*			g float8, h float8, i float8,
*			xoff float8, yoff float8, zoff float8,
*			xdimname text, ydimname text, zdimname text) returns pcpoint
*/
PG_FUNCTION_INFO_V1(lipoint_affine);
Datum lipoint_affine(PG_FUNCTION_ARGS)
{
	SERIALIZED_POINT *serpoint;
	PCPOINT *point;
	PCSCHEMA *schema;
	float8 a, b, c, d, e, f, g, h, i;
	float8 xoff, yoff, zoff;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpoint = PG_GETARG_SERPOINT_P(0);
	a = PG_GETARG_FLOAT8(1);
	b = PG_GETARG_FLOAT8(2);
	c = PG_GETARG_FLOAT8(3);
	d = PG_GETARG_FLOAT8(4);
	e = PG_GETARG_FLOAT8(5);
	f = PG_GETARG_FLOAT8(6);
	g = PG_GETARG_FLOAT8(7);
	h = PG_GETARG_FLOAT8(8);
	i = PG_GETARG_FLOAT8(9);
	xoff = PG_GETARG_FLOAT8(10);
	yoff = PG_GETARG_FLOAT8(11);
	zoff = PG_GETARG_FLOAT8(12);
	xdimname = PG_ARGISNULL(13) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(13));
	ydimname = PG_ARGISNULL(14) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(14));
	zdimname = PG_ARGISNULL(15) ? NULL : text_to_cstring(PG_GETARG_TEXT_P(15));

	schema = pc_schema_from_pcid(serpoint->pcid, fcinfo);

	point = pc_point_deserialize(serpoint, schema);
	if ( ! point )
	{
		elog(ERROR, "failed to deserialize point");
		PG_RETURN_NULL();
	}

	li_point_affine(point,
		a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
		xdimname, ydimname, zdimname);

	serpoint = pc_point_serialize(point);

	pc_point_free(point);

	PG_RETURN_POINTER(serpoint);
}

/**
* Apply an projective transformation to a point
* PC_Projective(patch pcpoint,
*			a float8, b float8, c float8, d float8,
*			e float8, f float8, g float8, h float8,
*			i float8, j float8, k float8, l float8,
*			m float8, n float8, o float8, p float8,
*			xdimname text, ydimname text, zdimname text) returns pcpoint
*/
PG_FUNCTION_INFO_V1(lipoint_projective);
Datum lipoint_projective(PG_FUNCTION_ARGS)
{
	SERIALIZED_POINT *serpoint;
	PCPOINT *point;
	PCSCHEMA *schema;
	float8 a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p;
	char *xdimname, *ydimname, *zdimname;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	serpoint = PG_GETARG_SERPOINT_P(0);
	a = PG_GETARG_FLOAT8(1);
	b = PG_GETARG_FLOAT8(2);
	c = PG_GETARG_FLOAT8(3);
	d = PG_GETARG_FLOAT8(4);
	e = PG_GETARG_FLOAT8(5);
	f = PG_GETARG_FLOAT8(6);
	g = PG_GETARG_FLOAT8(7);
	h = PG_GETARG_FLOAT8(8);
	i = PG_GETARG_FLOAT8(9);
	j = PG_GETARG_FLOAT8(10);
	k = PG_GETARG_FLOAT8(11);
	l = PG_GETARG_FLOAT8(12);
	m = PG_GETARG_FLOAT8(13);
	n = PG_GETARG_FLOAT8(14);
	o = PG_GETARG_FLOAT8(15);
	p = PG_GETARG_FLOAT8(16);
	xdimname = text_to_cstring(PG_GETARG_TEXT_P(17));
	ydimname = text_to_cstring(PG_GETARG_TEXT_P(18));
	zdimname = text_to_cstring(PG_GETARG_TEXT_P(19));

	schema = pc_schema_from_pcid(serpoint->pcid, fcinfo);

	point = pc_point_deserialize(serpoint, schema);
	if ( ! point )
	{
		elog(ERROR, "failed to deserialize point");
		PG_RETURN_NULL();
	}

	li_point_projective(point,
		a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p,
		xdimname, ydimname, zdimname);

	serpoint = pc_point_serialize(point);

	pc_point_free(point);

	PG_RETURN_POINTER(serpoint);
}

/**
* Apply an affine transformation to a box4d
* PC_Affine(box libox4d, mat43 float8[9]) returns libox4d
*/
PG_FUNCTION_INFO_V1(libox4d_affine_matr);
Datum libox4d_affine_matr(PG_FUNCTION_ARGS)
{
	LIBOX4 *ibox, *obox;
	float8 *mat43;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	ibox = (LIBOX4 *) PG_GETARG_POINTER(0);

	mat43 = li_getarg_float8_array(fcinfo, 1, 12);
	if ( ! mat43 )
		PG_RETURN_NULL();

	obox = li_box4d_affine(*ibox,
		mat43[0], mat43[1], mat43[2],
		mat43[4], mat43[5], mat43[6],
		mat43[8], mat43[9], mat43[10],
		mat43[3], mat43[7], mat43[11]);

	PG_RETURN_POINTER(obox);
}

/**
* Rotate and translate a box4d given a unit quaternion and a vector.
* PC_Affine(box libox4d, quat float8[4], vec float8[3]) returns libox4d
*/
PG_FUNCTION_INFO_V1(libox4d_affine_quat);
Datum libox4d_affine_quat(PG_FUNCTION_ARGS)
{
	int d;
	LIBOX4 *ibox, *obox;
	float8 *quat, *vec;

	if ( PG_ARGISNULL(0) )
		PG_RETURN_NULL();	/* returns null if no input values */

	ibox = (LIBOX4 *) PG_GETARG_POINTER(0);

	quat = li_getarg_float8_array(fcinfo, 1, 4);
	if ( ! quat )
		PG_RETURN_NULL();

	vec = li_getarg_float8_array(fcinfo, 2, 3);
	if ( ! vec )
		PG_RETURN_NULL();

	// rotate
	obox = li_box4d_rotate_quaternion(*ibox,
		quat[0], quat[1], quat[2], quat[3]);

	// translate
	for ( d = 0; d < 3; d++)
	{
		(*obox)[0][d] += vec[d];
		(*obox)[1][d] += vec[d];
	}

	PG_RETURN_POINTER(obox);
}
