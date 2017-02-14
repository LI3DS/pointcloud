/***********************************************************************
* pc_patch.c
*
*  Pointclound patch handling. Create, get and set values from the
*  basic PCPATCH structure.
*
*  PgSQL Pointcloud is free and open source software provided
*  by the Government of Canada
*  Copyright (c) 2013 Natural Resources Canada
*
***********************************************************************/

#include <math.h>
#include <assert.h>
#include "pc_api_internal.h"

int
pc_patch_compute_extent(PCPATCH *pa)
{
    if ( ! pa ) return PC_FAILURE;
    switch ( pa->type )
    {
    case PC_NONE:
        return pc_patch_uncompressed_compute_extent((PCPATCH_UNCOMPRESSED*)pa);
    case PC_GHT:
        return pc_patch_ght_compute_extent((PCPATCH_GHT*)pa);
    case PC_DIMENSIONAL:
        return pc_patch_dimensional_compute_extent((PCPATCH_DIMENSIONAL*)pa);
    case PC_LAZPERF:
        return pc_patch_lazperf_compute_extent((PCPATCH_LAZPERF*)pa);
    }
    return PC_FAILURE;
}

/**
* Calculate or re-calculate statistics for a patch.
*/
int
pc_patch_compute_stats(PCPATCH *pa)
{
    if ( ! pa ) return PC_FAILURE;

    switch ( pa->type )
    {
    case PC_NONE:
        return pc_patch_uncompressed_compute_stats((PCPATCH_UNCOMPRESSED*)pa);

    case PC_DIMENSIONAL:
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_dimensional((PCPATCH_DIMENSIONAL*)pa);
        pc_patch_uncompressed_compute_stats(pu);
        pa->stats = pu->stats; pu->stats = NULL;
        pc_patch_uncompressed_free(pu);
        return PC_SUCCESS;
    }
    case PC_GHT:
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_ght((PCPATCH_GHT*)pa);
        pc_patch_uncompressed_compute_stats(pu);
        pa->stats = pc_stats_clone(pu->stats);
        pc_patch_uncompressed_free(pu);
        return PC_SUCCESS;
    }
    case PC_LAZPERF:
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_lazperf((PCPATCH_LAZPERF*)pa);
        pc_patch_uncompressed_compute_stats(pu);
        pa->stats = pc_stats_clone(pu->stats);
        pc_patch_uncompressed_free(pu);
        return PC_SUCCESS;
    }
    default:
    {
        pcerror("%s: unknown compression type", __func__, pa->type);
        return PC_FAILURE;
    }
    }

    return PC_FAILURE;
}

void
pc_patch_free(PCPATCH *patch)
{
    if ( patch->stats )
    {
        pc_stats_free( patch->stats );
        patch->stats = NULL;
    }

    switch( patch->type )
    {
    case PC_NONE:
    {
        pc_patch_uncompressed_free((PCPATCH_UNCOMPRESSED*)patch);
        break;
    }
    case PC_GHT:
    {
        pc_patch_ght_free((PCPATCH_GHT*)patch);
        break;
    }
    case PC_DIMENSIONAL:
    {
        pc_patch_dimensional_free((PCPATCH_DIMENSIONAL*)patch);
        break;
    }
    case PC_LAZPERF:
    {
        pc_patch_lazperf_free((PCPATCH_LAZPERF*)patch);
        break;
    }
    default:
    {
        pcerror("%s: unknown compression type %d", __func__, patch->type);
        break;
    }
    }
}


PCPATCH *
pc_patch_from_pointlist(const PCPOINTLIST *ptl)
{
    return (PCPATCH*)pc_patch_uncompressed_from_pointlist(ptl);
}


PCPATCH *
pc_patch_compress(const PCPATCH *patch, void *userdata)
{
    uint32_t schema_compression = patch->schema->compression;
    uint32_t patch_compression = patch->type;

    switch ( schema_compression )
    {
    case PC_DIMENSIONAL:
    {
        if ( patch_compression == PC_NONE )
        {
            /* Dimensionalize, dimensionally compress, return */
            PCPATCH_DIMENSIONAL *pcdu = pc_patch_dimensional_from_uncompressed((PCPATCH_UNCOMPRESSED*)patch);
            PCPATCH_DIMENSIONAL *pcdd = pc_patch_dimensional_compress(pcdu, (PCDIMSTATS*)userdata);
            pc_patch_dimensional_free(pcdu);
            return (PCPATCH*)pcdd;
        }
        else if ( patch_compression == PC_DIMENSIONAL )
        {
            /* Make sure it's compressed, return */
            return (PCPATCH*)pc_patch_dimensional_compress((PCPATCH_DIMENSIONAL*)patch, (PCDIMSTATS*)userdata);
        }
        else if ( patch_compression == PC_GHT )
        {
            /* Uncompress, dimensionalize, dimensionally compress, return */
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_ght((PCPATCH_GHT*)patch);
            PCPATCH_DIMENSIONAL *pcdu  = pc_patch_dimensional_from_uncompressed(pcu);
            PCPATCH_DIMENSIONAL *pcdc  = pc_patch_dimensional_compress(pcdu, NULL);
            pc_patch_dimensional_free(pcdu);
            return (PCPATCH*)pcdc;
        }
        else if ( patch_compression == PC_LAZPERF )
        {
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_lazperf( (PCPATCH_LAZPERF*) patch );
            PCPATCH_DIMENSIONAL *pal = pc_patch_dimensional_from_uncompressed( pcu );
            PCPATCH_DIMENSIONAL *palc = pc_patch_dimensional_compress( pal, NULL );
            pc_patch_dimensional_free(pal);
            return (PCPATCH*) palc;
        }
        else
        {
            pcerror("%s: unknown patch compression type %d", __func__, patch_compression);
        }
    }
    case PC_NONE:
    {
        if ( patch_compression == PC_NONE )
        {
            return (PCPATCH*)patch;
        }
        else if ( patch_compression == PC_DIMENSIONAL )
        {
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_dimensional((PCPATCH_DIMENSIONAL*)patch);
            return (PCPATCH*)pcu;

        }
        else if ( patch_compression == PC_GHT )
        {
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_ght((PCPATCH_GHT*)patch);
            return (PCPATCH*)pcu;
        }
        else if ( patch_compression == PC_LAZPERF )
        {
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_lazperf( (PCPATCH_LAZPERF*)patch );
            return (PCPATCH*)pcu;
        }
        else
        {
            pcerror("%s: unknown patch compression type %d", __func__, patch_compression);
        }
    }
    case PC_GHT:
    {
        if ( patch_compression == PC_NONE )
        {
            PCPATCH_GHT *pgc = pc_patch_ght_from_uncompressed((PCPATCH_UNCOMPRESSED*)patch);
            if ( ! pgc ) pcerror("%s: ght compression failed", __func__);
            return (PCPATCH*)pgc;
        }
        else if ( patch_compression == PC_DIMENSIONAL )
        {
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_dimensional((PCPATCH_DIMENSIONAL*)patch);
            PCPATCH_GHT *pgc = pc_patch_ght_from_uncompressed((PCPATCH_UNCOMPRESSED*)patch);
            pc_patch_uncompressed_free(pcu);
            return (PCPATCH*)pgc;
        }
        else if ( patch_compression == PC_GHT )
        {
            return (PCPATCH*)patch;
        }
        else if ( patch_compression == PC_LAZPERF )
        {
            PCPATCH_LAZPERF *pal = pc_patch_lazperf_from_uncompressed((PCPATCH_UNCOMPRESSED*)patch);
            return (PCPATCH*)pal;
        }
        else
        {
            pcerror("%s: unknown patch compression type %d", __func__, patch_compression);
        }
    }
    case PC_LAZPERF:
    {
        if ( patch_compression == PC_NONE )
        {
            PCPATCH_LAZPERF *pgc = pc_patch_lazperf_from_uncompressed((PCPATCH_UNCOMPRESSED*)patch);
            if ( ! pgc ) pcerror("%s: lazperf compression failed", __func__);
            return (PCPATCH*)pgc;
        }
        else if ( patch_compression == PC_DIMENSIONAL )
        {
            PCPATCH_UNCOMPRESSED *pad = pc_patch_uncompressed_from_dimensional((PCPATCH_DIMENSIONAL*)patch);
            PCPATCH_LAZPERF *pal = pc_patch_lazperf_from_uncompressed( (PCPATCH_UNCOMPRESSED*) pad );
            pc_patch_uncompressed_free( pad );
            return (PCPATCH*)pal;
        }
        else if ( patch_compression == PC_GHT )
        {
            PCPATCH_UNCOMPRESSED *pcu = pc_patch_uncompressed_from_ght((PCPATCH_GHT*)patch);
            PCPATCH_LAZPERF *pal = pc_patch_lazperf_from_uncompressed((PCPATCH_UNCOMPRESSED*)patch);
            pc_patch_uncompressed_free(pcu);
            return (PCPATCH*)pcu;
        }
        else if ( patch_compression == PC_LAZPERF )
        {
            return (PCPATCH*)patch;
        }
        else
        {
            pcerror("%s: unknown patch compression type %d", __func__, patch_compression);
        }
    }
    default:
    {
        pcerror("%s: unknown schema compression type %d", __func__, schema_compression);
    }
    }

    pcerror("%s: fatal error", __func__);
    return NULL;
}


PCPATCH *
pc_patch_uncompress(const PCPATCH *patch)
{
    uint32_t patch_compression = patch->type;

    if ( patch_compression == PC_DIMENSIONAL )
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_dimensional((PCPATCH_DIMENSIONAL*)patch);
        return (PCPATCH*)pu;
    }

    if ( patch_compression == PC_NONE )
    {
        return (PCPATCH*)patch;
    }

    if ( patch_compression == PC_GHT )
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_ght((PCPATCH_GHT*)patch);
        return (PCPATCH*)pu;
    }

    if ( patch_compression == PC_LAZPERF )
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_lazperf( (PCPATCH_LAZPERF*)patch );
        return (PCPATCH*) pu;
    }

    return NULL;
}



PCPATCH *
pc_patch_from_wkb(const PCSCHEMA *s, uint8_t *wkb, size_t wkbsize)
{
    /*
    byte:     endianness (1 = NDR, 0 = XDR)
    uint32:   pcid (key to POINTCLOUD_SCHEMAS)
    uint32:   compression (0 = no compression, 1 = dimensional, 2 = GHT)
    uchar[]:  data (interpret relative to pcid and compression)
    */
    uint32_t compression, pcid;
    PCPATCH *patch;

    if ( ! wkbsize )
    {
        pcerror("%s: zero length wkb", __func__);
    }

    /*
    * It is possible for the WKB compression to be different from the
    * schema compression at this point. The schema compression is only
    * forced at serialization time.
    */
    pcid = wkb_get_pcid(wkb);
    compression = wkb_get_compression(wkb);

    if ( pcid != s->pcid )
    {
        pcerror("%s: wkb pcid (%d) not consistent with schema pcid (%d)", __func__, pcid, s->pcid);
    }

    switch ( compression )
    {
    case PC_NONE:
    {
        patch = pc_patch_uncompressed_from_wkb(s, wkb, wkbsize);
        break;
    }
    case PC_DIMENSIONAL:
    {
        patch = pc_patch_dimensional_from_wkb(s, wkb, wkbsize);
        break;
    }
    case PC_GHT:
    {
        patch = pc_patch_ght_from_wkb(s, wkb, wkbsize);
        break;
    }
    case PC_LAZPERF:
    {
        patch = pc_patch_lazperf_from_wkb(s, wkb, wkbsize);
        break;
    }
    default:
    {
        /* Don't get here */
        pcerror("%s: unknown compression '%d' requested", __func__, compression);
        return NULL;
    }
    }

    if ( PC_FAILURE == pc_patch_compute_extent(patch) )
        pcerror("%s: pc_patch_compute_extent failed", __func__);

    if ( PC_FAILURE == pc_patch_compute_stats(patch) )
        pcerror("%s: pc_patch_compute_stats failed", __func__);

    return patch;

}



uint8_t *
pc_patch_to_wkb(const PCPATCH *patch, size_t *wkbsize)
{
    /*
    byte:     endianness (1 = NDR, 0 = XDR)
    uint32:   pcid (key to POINTCLOUD_SCHEMAS)
    uint32:   compression (0 = no compression, 1 = dimensional, 2 = GHT)
    uchar[]:  data (interpret relative to pcid and compression)
    */
    switch ( patch->type )
    {
    case PC_NONE:
    {
        return pc_patch_uncompressed_to_wkb((PCPATCH_UNCOMPRESSED*)patch, wkbsize);
    }
    case PC_DIMENSIONAL:
    {
        return pc_patch_dimensional_to_wkb((PCPATCH_DIMENSIONAL*)patch, wkbsize);
    }
    case PC_GHT:
    {
        return pc_patch_ght_to_wkb((PCPATCH_GHT*)patch, wkbsize);
    }
    case PC_LAZPERF:
    {
        return pc_patch_lazperf_to_wkb((PCPATCH_LAZPERF*)patch, wkbsize);
    }
    }
    pcerror("%s: unknown compression requested '%d'", __func__, patch->schema->compression);
    return NULL;
}

char *
pc_patch_to_string(const PCPATCH *patch)
{
    switch( patch->type )
    {
    case PC_NONE:
        return pc_patch_uncompressed_to_string((PCPATCH_UNCOMPRESSED*)patch);
    case PC_DIMENSIONAL:
        return pc_patch_dimensional_to_string((PCPATCH_DIMENSIONAL*)patch);
    case PC_GHT:
        return pc_patch_ght_to_string((PCPATCH_GHT*)patch);
    case PC_LAZPERF:
        return pc_patch_lazperf_to_string( (PCPATCH_LAZPERF*)patch );
    }
    pcerror("%s: unsupported compression %d requested", __func__, patch->type);
    return NULL;
}





PCPATCH *
pc_patch_from_patchlist(PCPATCH **palist, int numpatches)
{
    int i;
    uint32_t totalpoints = 0;
    PCPATCH_UNCOMPRESSED *paout;
    const PCSCHEMA *schema = NULL;
    uint8_t *buf;

    assert(palist);
    assert(numpatches);

    /* All schemas better be the same... */
    schema = palist[0]->schema;

    /* How many points will this output have? */
    for ( i = 0; i < numpatches; i++ )
    {
        if ( schema->pcid != palist[i]->schema->pcid )
        {
            pcerror("%s: inconsistent schemas in input", __func__);
            return NULL;
        }
        totalpoints += palist[i]->npoints;
    }

    /* Blank output */
    paout = pc_patch_uncompressed_make(schema, totalpoints);
    buf = paout->data;

    /* Uncompress dimensionals, copy uncompressed */
    for ( i = 0; i < numpatches; i++ )
    {
        const PCPATCH *pa = palist[i];

        /* Update bounds */
        pc_bounds_merge(&(paout->bounds), &(pa->bounds));

        switch ( pa->type )
        {
        case PC_DIMENSIONAL:
        {
            PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_dimensional((const PCPATCH_DIMENSIONAL*)pa);
            size_t sz = pu->schema->size * pu->npoints;
            memcpy(buf, pu->data, sz);
            buf += sz;
            pc_patch_free((PCPATCH*)pu);
            break;
        }
        case PC_GHT:
        {
            PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_ght((const PCPATCH_GHT*)pa);
            size_t sz = pu->schema->size * pu->npoints;
            memcpy(buf, pu->data, sz);
            buf += sz;
            pc_patch_uncompressed_free(pu);
            break;
        }
        case PC_NONE:
        {
            PCPATCH_UNCOMPRESSED *pu = (PCPATCH_UNCOMPRESSED*)pa;
            size_t sz = pu->schema->size * pu->npoints;
            memcpy(buf, pu->data, sz);
            buf += sz;
            break;
        }
        case PC_LAZPERF:
        {
            PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_lazperf((const PCPATCH_LAZPERF*)pa);
            size_t sz = pu->schema->size * pu->npoints;
            memcpy(buf, pu->data, sz);
            buf += sz;
            pc_patch_uncompressed_free(pu);
            break;
        }
        default:
        {
            pcerror("%s: unknown compression type (%d)", __func__, pa->type);
            break;
        }
        }
    }

    paout->npoints = totalpoints;

    if ( PC_FAILURE == pc_patch_uncompressed_compute_stats(paout) )
    {
        pcerror("%s: stats computation failed", __func__);
        return NULL;
    }

    return (PCPATCH*)paout;
}

// first: the first element to select.
// count: the number of element to select from the first one.
PCPATCH *
pc_patch_range(const PCPATCH *pa, int first, int count)
{
    PCPATCH_UNCOMPRESSED *paout;
    uint8_t *buf;
    size_t size;
    size_t start;

    assert(pa);

    if(first+count > pa->npoints)
        count = pa->npoints - first;
    if ( count <= 0 )
        return NULL;

    paout = pc_patch_uncompressed_make(pa->schema, count);
    buf = paout->data;
    paout->npoints = count;
    start = pa->schema->size * first;
    size = pa->schema->size * count;

    switch( pa->type )
    {
    case PC_NONE:
    {
        PCPATCH_UNCOMPRESSED *pu = (PCPATCH_UNCOMPRESSED*)pa;
        memcpy(buf, pu->data + start, size);
        break;
    }
    case PC_DIMENSIONAL:
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_dimensional((const PCPATCH_DIMENSIONAL*)pa);
        memcpy(buf, pu->data + start, size);
        pc_patch_free((PCPATCH*)pu);
        break;
    }
    case PC_GHT:
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_ght((const PCPATCH_GHT*)pa);
        memcpy(buf, pu->data + start, size);
        pc_patch_free((PCPATCH*)pu);
        break;
    }
    case PC_LAZPERF:
    {
        PCPATCH_UNCOMPRESSED *pu = pc_patch_uncompressed_from_lazperf((const PCPATCH_LAZPERF*)pa);
        memcpy(buf, pu->data + start, size);
        pc_patch_free((PCPATCH*)pu);
        break;
    }
    default:
    {
        pcerror("%s: unknown compression type (%d)", __func__, pa->type);
        break;
    }
    }

    if ( PC_FAILURE == pc_patch_uncompressed_compute_stats(paout) )
    {
        pcerror("%s: stats computation failed", __func__);
        return NULL;
    }

    return (PCPATCH*) paout;
}

/** get point n from patch */
/** positive 1-based:  1=first point,  npoints=last  point */
/** negative 1-based: -1=last  point, -npoints=first point */
PCPOINT *pc_patch_pointn(const PCPATCH *patch, int n)
{
    if(!patch) return NULL;
    if(n<0) n = patch->npoints+n; // negative indices count a backward
    else --n; // 1-based => 0-based indexing
    if(n<0 || n>= patch->npoints) return NULL;

    switch( patch->type )
    {
    case PC_NONE:
        return pc_patch_uncompressed_pointn((PCPATCH_UNCOMPRESSED*)patch,n);
    case PC_DIMENSIONAL:
        return pc_patch_dimensional_pointn((PCPATCH_DIMENSIONAL*)patch,n);
    case PC_GHT:
        return pc_patch_ght_pointn((PCPATCH_GHT*)patch,n);
    }
    pcerror("%s: unsupported compression %d requested", __func__, patch->type);
    return NULL;
}

#define PC_PATCH_SET_SCHEMA_COPY_RAW_VALUES 1
#define PC_PATCH_SET_SCHEMA_MEMCPY 1

#if PC_PATCH_SET_SCHEMA_MEMCPY

typedef struct
{
  uint32_t size;
  uint32_t obyteoffset;
  uint32_t nbyteoffset;
} PCMEMCPY;

void pc_point_copy( const PCPOINT *opt, const PCMEMCPY *cpy, uint32_t ncpy, PCPOINT *npt )
{
  int j;
  for ( j = 0; j < ncpy; j++ )
  {
 // do not interpret the value, copy the raw payload
 // defaults to 0 for the raw value
        char *ndata = npt->data + cpy[j].nbyteoffset;
        char *odata = opt->data + cpy[j].obyteoffset;
        memcpy(ndata,odata,cpy[j].size);
  }
}


#else

void pc_point_copy( const PCPOINT *in, const PCDIMENSION **dims, PCPOINT *out )
{
  int j;
  for ( j = 0; j < out->schema->ndims; j++ )
  {
#if PC_PATCH_SET_SCHEMA_COPY_RAW_VALUES
 // do not interpret the value, copy the raw payload
 // defaults to 0 for the raw value
    if ( dims[j] )
    {
        char *dataout = out->data + out->schema->dims[j]->byteoffset;
        char *datain  = in ->data +              dims[j]->byteoffset;
        memcpy(dataout,datain,dims[j]->size);
    }
#else
    // interpret the value, apply the scale/offset change
    // defaults to 0 for the interpreted value
    double val = 0.0;
    if ( dims[j] )
        pc_point_get_double(in, dims[j], &val);

    pc_point_set_double(out, out->schema->dims[j], val);
#endif
  }
}

#endif


/** set schema for patch */
PCPATCH*
pc_patch_set_schema(const PCPATCH *patch, const PCSCHEMA *new_schema)
{
    PCPATCH *pain = NULL;
    PCPATCH_UNCOMPRESSED *paout = NULL;
    PCPOINT opt, npt;
    double val;
    size_t i;
    char *name;

    // Get the reordered input dims
#if PC_PATCH_SET_SCHEMA_MEMCPY
    const PCDIMENSION *dim = NULL;
    PCMEMCPY *mcpy = pcalloc(new_schema->ndims * sizeof(PCMEMCPY));
    uint32_t m = 0;
#else
    const PCDIMENSION **dims = pcalloc(new_schema->ndims * sizeof(PCDIMENSION *));
#endif
    for ( i = 0; i < new_schema->ndims; i++ )
    {
      name = new_schema->dims[i]->name;
#if PC_PATCH_SET_SCHEMA_MEMCPY
      dim = pc_schema_get_dimension_by_name(patch->schema, name);
      if(dim && dim->interpretation != new_schema->dims[i]->interpretation)
          pcerror("%s : interpretations are not matching",__func__);
      // if possible, coalesce memcpys
      if(m == 0 || mcpy[m-1].obyteoffset+mcpy[m-1].size != dim->byteoffset)
      {
        mcpy[m].obyteoffset = dim->byteoffset;
        mcpy[m].nbyteoffset = new_schema->dims[i]->byteoffset;
        mcpy[m].size = dim->size;
        ++m;
      }
      else
      {
        mcpy[m-1].size += dim->size;
      }
#else
      dims[i] = pc_schema_get_dimension_by_name(patch->schema, name);
#endif
    }

    // prepare uncompressed input and output patches
    pain  = pc_patch_uncompress(patch);
    paout = pc_patch_uncompressed_make(new_schema,patch->npoints);
    paout->npoints = pain->npoints;

    // setup input and output points
    opt.schema = patch->schema;
    npt.schema = new_schema;
    opt.readonly = PC_TRUE;
    npt.readonly = PC_TRUE;

    // process the point set
    opt.data = ((PCPATCH_UNCOMPRESSED *)pain)->data;
    npt.data = paout->data;

#if PC_PATCH_SET_SCHEMA_MEMCPY
    if( m==0 )
    {
      // all dimensions are new, no copy
    }
    else if(opt.schema->size==npt.schema->size && mcpy[0].size==npt.schema->size)
    {
      // memcpy of the entire patch coalesces to a single memcpy of the whole data
        memcpy(npt.data,opt.data,npt.schema->size*patch->npoints);
pcinfo("entire ! %d %d %d %d %d\n",mcpy[0].nbyteoffset,mcpy[0].obyteoffset,mcpy[0].size,npt.schema->size,opt.schema->size);
    }
    else for ( i = 0; i <patch->npoints; i++ )
    {
      // fragmented memcpys
      pc_point_copy(&opt,mcpy,m,&npt);
      opt.data += opt.schema->size;
      npt.data += npt.schema->size;
    }
#else
    for ( i = 0; i <patch->npoints; i++ )
    {
      pc_point_copy(&opt,dims,&npt);
      opt.data += opt.schema->size;
      npt.data += npt.schema->size;
    }
#endif

    // update the stats and bounds
    if(patch->stats)
    {
      paout->stats = pc_stats_new(new_schema);

      opt.data = patch->stats->min.data;
      npt.data = paout->stats->min.data;
  #if PC_PATCH_SET_SCHEMA_MEMCPY
        pc_point_copy(&opt,mcpy,m,&npt);
  #else
        pc_point_copy(&opt,dims,&npt);
  #endif

      opt.data = patch->stats->max.data;
      npt.data = paout->stats->max.data;
  #if PC_PATCH_SET_SCHEMA_MEMCPY
        pc_point_copy(&opt,mcpy,m,&npt);
  #else
        pc_point_copy(&opt,dims,&npt);
  #endif

      opt.data = patch->stats->avg.data;
      npt.data = paout->stats->avg.data;
  #if PC_PATCH_SET_SCHEMA_MEMCPY
        pc_point_copy(&opt,mcpy,m,&npt);
  #else
        pc_point_copy(&opt,dims,&npt);
  #endif

#if PC_PATCH_SET_SCHEMA_COPY_RAW_VALUES
      paout->bounds.xmin = pc_point_get_x(&paout->stats->min);
      paout->bounds.ymin = pc_point_get_y(&paout->stats->min);
      paout->bounds.xmax = pc_point_get_x(&paout->stats->max);
      paout->bounds.ymax = pc_point_get_y(&paout->stats->max);
    }
    else
    {
      PCDIMENSION *odimx = opt.schema->dims[opt.schema->x_position];
      PCDIMENSION *odimy = opt.schema->dims[opt.schema->y_position];
      PCDIMENSION *ndimx = npt.schema->dims[opt.schema->x_position];
      PCDIMENSION *ndimy = npt.schema->dims[opt.schema->y_position];

      double xscale = ndimx->scale / odimx->scale;
      double yscale = ndimy->scale / odimy->scale;
      double xoffset = ndimx->offset - xscale * odimx->offset;
      double yoffset = ndimy->offset - yscale * odimy->offset;

      paout->bounds.xmin = patch->bounds.xmin*xscale + xoffset;
      paout->bounds.xmax = patch->bounds.xmax*xscale + xoffset;
      paout->bounds.ymin = patch->bounds.ymin*yscale + yoffset;
      paout->bounds.ymax = patch->bounds.ymax*yscale + yoffset;
    }
#else
    }
    paout->bounds = patch->bounds;
#endif


#if PC_PATCH_SET_SCHEMA_MEMCPY
    pcfree(mcpy);
#else
    pcfree(dims);
#endif

    if(pain != patch)
      pc_patch_free(pain);

    return (PCPATCH*) paout;
}

/**
* Rotate a patch given a unit quaternion.
*/
PCPATCH *
pc_patch_rotate_quaternion(
    const PCPATCH *patch,
    double qw, double qx, double qy, double qz,
    const char *xdimname, const char *ydimname, const char *zdimname)
{
    PCPATCH_UNCOMPRESSED *uncompressed_patch;
    PCDIMENSION *xdim, *ydim, *zdim;
    const PCSCHEMA *schema;
    PCPOINTLIST *pointlist;
    PCMAT33 qmat;
    PCVEC3 vec, rvec;
    size_t i;

    pc_matrix_set_from_quaternion(qmat, qw, qx, qy, qz);

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

    xdim = pc_schema_get_dimension_by_name(schema, xdimname);
    ydim = pc_schema_get_dimension_by_name(schema, ydimname);
    zdim = pc_schema_get_dimension_by_name(schema, zdimname);

    pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

    // the points of pointlist include pointers to the uncompressed patch data
    // block, so updating the points changes the patch payload

    for ( i = 0; i < pointlist->npoints; i++ )
    {
        PCPOINT *point = pc_pointlist_get_point(pointlist, i);

        pc_point_get_double(point, xdim, &vec[0]);
        pc_point_get_double(point, ydim, &vec[1]);
        pc_point_get_double(point, zdim, &vec[2]);

        pc_matrix_multiply_vector(rvec, qmat, vec);

        pc_point_set_double(point, xdim, rvec[0]);
        pc_point_set_double(point, ydim, rvec[1]);
        pc_point_set_double(point, zdim, rvec[2]);
    }

    pc_pointlist_free(pointlist);

    return ((PCPATCH *)uncompressed_patch);
}

/**
* Translate a patch.
*/
PCPATCH *
pc_patch_translate(
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

    xdim = pc_schema_get_dimension_by_name(schema, xdimname);
    ydim = pc_schema_get_dimension_by_name(schema, ydimname);
    zdim = pc_schema_get_dimension_by_name(schema, zdimname);

    pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

    // the points of pointlist include pointers to the uncompressed patch data
    // block, so updating the points changes the patch payload

    for ( i = 0; i < pointlist->npoints; i++ )
    {
        PCPOINT *point = pc_pointlist_get_point(pointlist, i);

        pc_point_get_double(point, xdim, &x);
        pc_point_get_double(point, ydim, &y);
        pc_point_get_double(point, zdim, &z);

        pc_point_set_double(point, xdim, x + tx);
        pc_point_set_double(point, ydim, y + ty);
        pc_point_set_double(point, zdim, z + tz);
    }

    pc_pointlist_free(pointlist);

    return ((PCPATCH *)uncompressed_patch);
}

/**
* Apply an affine transformation to a patch.
*/
PCPATCH *
pc_patch_affine(
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
    PCMAT43 amat;
    PCVEC3 vec, rvec;
    size_t idx;

    pc_matrix_set_affine(amat, a, b, c, d, e, f, g, h, i, xoff, yoff, zoff);

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

    xdim = pc_schema_get_dimension_by_name(schema, xdimname);
    ydim = pc_schema_get_dimension_by_name(schema, ydimname);
    zdim = pc_schema_get_dimension_by_name(schema, zdimname);

    pointlist = pc_pointlist_from_uncompressed(uncompressed_patch);

    // the points of pointlist include pointers to the uncompressed patch data
    // block, so updating the points changes the patch payload

    for ( idx = 0; idx < pointlist->npoints; idx++ )
    {
        PCPOINT *point = pc_pointlist_get_point(pointlist, idx);

        pc_point_get_double(point, xdim, &vec[0]);
        pc_point_get_double(point, ydim, &vec[1]);
        pc_point_get_double(point, zdim, &vec[2]);

        pc_matrix_transform_affine(rvec, amat, vec);

        pc_point_set_double(point, xdim, rvec[0]);
        pc_point_set_double(point, ydim, rvec[1]);
        pc_point_set_double(point, zdim, rvec[2]);
    }

    pc_pointlist_free(pointlist);

    return ((PCPATCH *)uncompressed_patch);
}
