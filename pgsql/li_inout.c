/***********************************************************************
* li_inout.c
*
***********************************************************************/

#include "pc_pgsql.h"      /* Common PgSQL support for our type */

Datum lifrustum_from_patch_as_bytea(PG_FUNCTION_ARGS);

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
