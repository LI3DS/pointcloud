/***********************************************************************
* pc_editor.c
*
*  Editor functions for points and patches in PgSQL.
*
*  PgSQL Pointcloud is free and open source software provided
*  by the Government of Canada
*  Copyright (c) 2013 Natural Resources Canada
*  Copyright (c) 2017 Oslandia
*
***********************************************************************/

#include "pc_pgsql.h"	   /* Common PgSQL support for our type */

Datum pcpatch_rotate_quaternion(PG_FUNCTION_ARGS);

/**
* Rotate a patch based on a rotation quaternion
* PC_RotateQuaternion(patch pcpatch, qw float8, qx float8, qy float8, qz float8,
*					  xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(pcpatch_rotate_quaternion);
Datum pcpatch_rotate_quaternion(PG_FUNCTION_ARGS)
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

	patch_out = pc_patch_rotate_quaternion(
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
PG_FUNCTION_INFO_V1(pcpatch_translate);
Datum pcpatch_translate(PG_FUNCTION_ARGS)
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

	patch_out = pc_patch_translate(
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
PG_FUNCTION_INFO_V1(pcpatch_affine);
Datum pcpatch_affine(PG_FUNCTION_ARGS)
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

	patch_out = pc_patch_affine(patch_in,
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
PG_FUNCTION_INFO_V1(pcpatch_projective);
Datum pcpatch_projective(PG_FUNCTION_ARGS)
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

	patch_out = pc_patch_projective(patch_in,
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
PG_FUNCTION_INFO_V1(pcpoint_rotate_quaternion);
Datum pcpoint_rotate_quaternion(PG_FUNCTION_ARGS)
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

	pc_point_rotate_quaternion(
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
PG_FUNCTION_INFO_V1(pcpoint_translate);
Datum pcpoint_translate(PG_FUNCTION_ARGS)
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

	pc_point_translate(
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
PG_FUNCTION_INFO_V1(pcpoint_affine);
Datum pcpoint_affine(PG_FUNCTION_ARGS)
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

	pc_point_affine(point,
		a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
		xdimname, ydimname, zdimname);

	serpoint = pc_point_serialize(point);

	pc_point_free(point);

	PG_RETURN_POINTER(serpoint);
}
