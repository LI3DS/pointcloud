/***********************************************************************
* li_inout.c
*
***********************************************************************/

#include "pc_pgsql.h"      /* Common PgSQL support for our type */

#define SHOW_DIGS_DOUBLE 15
#define MAX_DIGS_DOUBLE (SHOW_DIGS_DOUBLE + 6 + 1 + 3 + 1)

Datum libox4d_in(PG_FUNCTION_ARGS);

Datum libox4d_as_box3ds(PG_FUNCTION_ARGS);
Datum libox4d_as_array(PG_FUNCTION_ARGS);
Datum lifrustum_from_patch_as_bytea(PG_FUNCTION_ARGS);


PG_FUNCTION_INFO_V1(libox4d_in);
Datum libox4d_in(PG_FUNCTION_ARGS)
{
	int nitems;
	char *str = PG_GETARG_CSTRING(0);
	LIBOX4 *box = palloc(sizeof(LIBOX4));
	memset(*box, 0, sizeof(LIBOX4));

	if ( strstr(str, "BOX4D(") == str )
	{
		nitems = sscanf(str, "BOX4D(%le %le %le %le ,%le %le %le %le)",
						&((*box)[0][0]), &((*box)[0][1]), &((*box)[0][2]), &((*box)[0][3]),
						&((*box)[1][0]), &((*box)[1][1]), &((*box)[1][2]), &((*box)[1][3]));
		if ( nitems != 8 )
		{
			nitems = sscanf(str, "BOX4D(%le %le %le ,%le %le %le)",
							&((*box)[0][0]), &((*box)[0][1]), &((*box)[0][2]),
							&((*box)[1][0]), &((*box)[1][1]), &((*box)[1][2]));
			if ( nitems != 6 )
			{
				nitems = sscanf(str, "BOX4D(%le %le ,%le %le)",
								&((*box)[0][0]), &((*box)[0][1]),
								&((*box)[1][0]), &((*box)[1][1]));
				if ( nitems != 4 )
				{
					pfree(box);
					elog(ERROR, "BOX4D parser - couldn't parse");
					PG_RETURN_NULL();
				}
			}
		}
	}
	else if ( strstr(str, "BOX3D(") == str )
	{
		nitems = sscanf(str, "BOX3D(%le %le %le, %le %le %le)",
							&((*box)[0][0]), &((*box)[0][1]), &((*box)[0][2]),
							&((*box)[1][0]), &((*box)[1][1]), &((*box)[1][2]));
		if ( nitems != 6 )
		{
			nitems = sscanf(str, "BOX3D(%le %le ,%le %le)",
							&((*box)[0][0]), &((*box)[0][1]),
							&((*box)[1][0]), &((*box)[1][1]));
			if ( nitems != 4 )
			{
				pfree(box);
				elog(ERROR, "BOX4D parser - couldn't parse");
				PG_RETURN_NULL();
			}
		}
	}
	else
	{
		pfree(box);
		elog(ERROR,"BOX4D parser - doesn't start with BOX4D( or BOX3D(");
		PG_RETURN_NULL();
	}

	if ( (*box)[0][0] > (*box)[1][0] )
	{
		float tmp = (*box)[0][0];
		(*box)[0][0] = (*box)[1][0];
		(*box)[1][0] = tmp;
	}
	if ( (*box)[0][1] > (*box)[1][1] )
	{
		float tmp = (*box)[0][1];
		(*box)[0][1] = (*box)[1][1];
		(*box)[1][1] = tmp;
	}
	if ( (*box)[0][2] > (*box)[1][2] )
	{
		float tmp = (*box)[0][2];
		(*box)[0][2] = (*box)[1][2];
		(*box)[1][2] = tmp;
	}
	if ( (*box)[0][3] > (*box)[1][3] )
	{
		float tmp = (*box)[0][3];
		(*box)[0][3] = (*box)[1][3];
		(*box)[1][3] = tmp;
	}

	PG_RETURN_POINTER(box);
}

PG_FUNCTION_INFO_V1(libox4d_out);
Datum libox4d_out(PG_FUNCTION_ARGS)
{
	LIBOX4 *box = (LIBOX4 *)PG_GETARG_POINTER(0);
	int size;
	char *result;

	if (box == NULL)
	{
		result = palloc(5);
		strcat(result, "NULL");
		PG_RETURN_CSTRING(result);
	}

	/* double digits + "BOX4D" + "()" + spaces + comma + null */
	size = MAX_DIGS_DOUBLE*8 + 5 + 2 + 6 + 1 + 1;

	result = palloc(size);

	sprintf(result, "BOX4D(%.15g %.15g %.15g %.15g,%.15g %.15g %.15g %.15g)",
			(*box)[0][0], (*box)[0][1], (*box)[0][2], (*box)[0][3],
			(*box)[1][0], (*box)[1][1], (*box)[1][2], (*box)[1][3]);

	PG_RETURN_CSTRING(result);
}

PG_FUNCTION_INFO_V1(libox4d_as_box3d);
Datum libox4d_as_box3d(PG_FUNCTION_ARGS)
{
	LIBOX4 *box = (LIBOX4 *)PG_GETARG_POINTER(0);
	char *str;
	text *txt;

	str = li_box4d_as_box3d(*box);
	txt = cstring_to_text(str);
	pfree(str);

	PG_RETURN_TEXT_P(txt);
}

PG_FUNCTION_INFO_V1(libox4d_as_array);
Datum libox4d_as_array(PG_FUNCTION_ARGS)
{
	LIBOX4 *box = (LIBOX4 *)PG_GETARG_POINTER(0);
	ArrayType *result;
	Datum *elems;
	double *vals;
	size_t i;
	static size_t n = 8;

	elems = (Datum *)palloc(n * sizeof(Datum));
	vals = li_box4d_to_double_array(*box);
	i = n;
	while (i--) elems[i] = Float8GetDatum(vals[i]);
	pcfree(vals);
	result = construct_array(elems, n, FLOAT8OID,
			sizeof(float8), FLOAT8PASSBYVAL, 'd');

	PG_RETURN_ARRAYTYPE_P(result);
}

PG_FUNCTION_INFO_V1(lifrustum_from_patch_as_bytea);
Datum lifrustum_from_patch_as_bytea(PG_FUNCTION_ARGS)
{
	uint8 *bytes;
	size_t bytes_size;
	bytea *wkb;
	size_t wkb_size;
	SERIALIZED_PATCH *serpatch = PG_GETHEADER_SERPATCH_P(0);

	// TODO: use directly PCBOUNDS when it will be 3D
	PCBOUNDS *b = & serpatch->bounds;
	const LIBOX3 box = {
		{ b->xmin, b->ymin, 0 },
		{ b->xmax, b->ymax, 10 }
	};

	LIFRUSTUM f;
	uint32_t srid = 0;

	li_frustum_from_box(&f,box);
	bytes = li_frustum_to_geometry_wkb(&f, srid, &bytes_size);
	wkb_size = VARHDRSZ + bytes_size;
	wkb = palloc(wkb_size);
	memcpy(VARDATA(wkb), bytes, bytes_size);
	SET_VARSIZE(wkb, wkb_size);

	pfree(bytes);

	PG_RETURN_BYTEA_P(wkb);
}
