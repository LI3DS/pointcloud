/***********************************************************************
* cu_li_patch.c
*
***********************************************************************/

#include <math.h>
#include "CUnit/Basic.h"
#include "cu_tester.h"

/* GLOBALS ************************************************************/

static PCSCHEMA *simpleschema = NULL;
static const char *simplexmlfile = "data/simple-schema.xml";

/* Setup/teardown for this suite */
static int
init_suite(void)
{
	char *xmlstr = file_to_str(simplexmlfile);
	simpleschema = pc_schema_from_xml(xmlstr);
	pcfree(xmlstr);
	if ( !simpleschema ) return 1;
	return 0;
}

static int
clean_suite(void)
{
	pc_schema_free(simpleschema);
	return 0;
}


/* TESTS **************************************************************/

static void
test_patch_rotate_quaternion_compression_none()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double angle;
	double qw, qx, qy, qz;
	double v;

	// (1, 1, 1) is the point we're going to rotate
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	pc_pointlist_add_point(pl, pt);

	// π/2 rotation around x axis
	// expected result: (1, -1, 1)
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = sin(angle / 2.);
	qy = 0;
	qz = 0;
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_uncompressed, qw, qx, qy, qz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == -1);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == 1);
	CU_ASSERT(patch->bounds.xmax == 1);
	CU_ASSERT(patch->bounds.ymin == -1);
	CU_ASSERT(patch->bounds.ymax == -1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	// π/2 rotation around y axis
	// expected result: (1, 1, -1)
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = sin(angle / 2.);
	qz = 0;
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_uncompressed, qw, qx, qy, qz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == -1);
	CU_ASSERT(patch->bounds.xmin == 1);
	CU_ASSERT(patch->bounds.xmax == 1);
	CU_ASSERT(patch->bounds.ymin == 1);
	CU_ASSERT(patch->bounds.ymax == 1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	// π/2 rotation around z axis
	// expected result: (-1, 1, 1)
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = 0;
	qz = sin(angle / 2.);
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_uncompressed, qw, qx, qy, qz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == -1);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == -1);
	CU_ASSERT(patch->bounds.xmax == -1);
	CU_ASSERT(patch->bounds.ymin == 1);
	CU_ASSERT(patch->bounds.ymax == 1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

static void
test_patch_rotate_quaternion_compression_none_with_dimension_names()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double angle;
	double qw, qx, qy, qz;
	double v;

	// this is the same test as test_patch_rotate_quaternion, but explict
	// dimension names are passed to li_patch_rotate_quaternion

	// (1, 1, 1) is the point we're going to rotate
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
	pc_pointlist_add_point(pl, pt);

	// π/2 rotation around z axis
	// expected result: (-1, 1, 1)
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = 0;
	qz = sin(angle / 2.);
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_uncompressed, qw, qx, qy, qz, "x", "y", "z");
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == -1);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == 1);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == -1);
	CU_ASSERT(patch->bounds.xmax == -1);
	CU_ASSERT(patch->bounds.ymin == 1);
	CU_ASSERT(patch->bounds.ymax == 1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}


#ifdef HAVE_LIBGHT
static void
test_patch_rotate_quaternion_compression_ght()
{
	PCPATCH *patch;
	PCPATCH_GHT *patch_ght;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double angle;
	double qw, qx, qy, qz;
	double v;

	// (1, 1, 1) is the point we're going to rotate
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	pc_pointlist_add_point(pl, pt);

	// π/2 rotation around x axis
	// expected result: (1, -1, 1)
	patch_ght = pc_patch_ght_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = sin(angle / 2.);
	qy = 0;
	qz = 0;
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_ght, qw, qx, qy, qz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == -1);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == 1);
	CU_ASSERT(patch->bounds.xmax == 1);
	CU_ASSERT(patch->bounds.ymin == -1);
	CU_ASSERT(patch->bounds.ymax == -1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH *)patch_ght);

	// π/2 rotation around y axis
	// expected result: (1, 1, -1)
	patch_ght = pc_patch_ght_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = sin(angle / 2.);
	qz = 0;
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_ght, qw, qx, qy, qz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == -1);
	CU_ASSERT(patch->bounds.xmin == 1);
	CU_ASSERT(patch->bounds.xmax == 1);
	CU_ASSERT(patch->bounds.ymin == 1);
	CU_ASSERT(patch->bounds.ymax == 1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH *)patch_ght);

	// π/2 rotation around z axis
	// expected result: (-1, 1, 1)
	patch_ght = pc_patch_ght_from_pointlist(pl);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = 0;
	qz = sin(angle / 2.);
	patch = li_patch_rotate_quaternion(
			(PCPATCH *)patch_ght, qw, qx, qy, qz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == -1);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == -1);
	CU_ASSERT(patch->bounds.xmax == -1);
	CU_ASSERT(patch->bounds.ymin == 1);
	CU_ASSERT(patch->bounds.ymax == 1);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH *)patch_ght);

	pc_pointlist_free(pl);
}
#endif	/* HAVE_LIBGHT */

static void
test_patch_translate_compression_none()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double tx, ty, tz;
	double v;

	// (1, 1, 1) is the point we're going to translate
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	pc_pointlist_add_point(pl, pt);

	// (-1, 1, 2) translation
	// expected result: (0, 2, 3)
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	tx = -1.0;
	ty = 1.0;
	tz = 2.0;
	patch = li_patch_translate(
			(PCPATCH *)patch_uncompressed, tx, ty, tz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 0);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 2);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 3);
	CU_ASSERT(patch->bounds.xmin == 0);
	CU_ASSERT(patch->bounds.xmax == 0);
	CU_ASSERT(patch->bounds.ymin == 2);
	CU_ASSERT(patch->bounds.ymax == 2);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

static void
test_patch_translate_compression_none_with_dimension_names()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double tx, ty, tz;
	double v;

	// this is the same test as test_patch_translate, but explict
	// dimension names are passed to li_patch_translate

	// (1, 1, 1) is the point we're going to translate
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
	pc_pointlist_add_point(pl, pt);

	// (-1, 1, 2) translation
	// expected result: (0, 2, 3)
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	tx = -1.0;
	ty = 1.0;
	tz = 2.0;
	patch = li_patch_translate(
			(PCPATCH *)patch_uncompressed, tx, ty, tz, "x", "y", "z");
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == 0);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == 2);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 3);
	CU_ASSERT(patch->bounds.xmin == 0);
	CU_ASSERT(patch->bounds.xmax == 0);
	CU_ASSERT(patch->bounds.ymin == 2);
	CU_ASSERT(patch->bounds.ymax == 2);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

#ifdef HAVE_LIBGHT
static void
test_patch_translate_compression_ght()
{
	PCPATCH *patch;
	PCPATCH_GHT *patch_ght;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double tx, ty, tz;
	double v;

	// (1, 1, 1) is the point we're going to translate
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	pc_pointlist_add_point(pl, pt);

	// (-1, 1, 2) translation
	// expected result: (0, 2, 3)
	patch_ght = pc_patch_ght_from_pointlist(pl);
	tx = -1.0;
	ty = 1.0;
	tz = 2.0;
	patch = li_patch_translate(
			(PCPATCH *)patch_ght, tx, ty, tz, NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 0);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 2);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == 3);
	CU_ASSERT(patch->bounds.xmin == 0);
	CU_ASSERT(patch->bounds.xmax == 0);
	CU_ASSERT(patch->bounds.ymin == 2);
	CU_ASSERT(patch->bounds.ymax == 2);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH *)patch_ght);

	pc_pointlist_free(pl);
}
#endif	/* HAVE_LIBGHT */

static void
test_patch_affine_compression_none()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double a, b, c, d, e, f, g, h, i;
	double xoff, yoff, zoff;
	double v;

	// (1, 1, 1) is the point we're going to transform
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	pc_pointlist_add_point(pl, pt);

	// scale + translate
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	a = 2;
	b = 0;
	c = 0;
	d = 0;
	e = 2;
	f = 0;
	g = 0;
	h = 0;
	i = 2;
	xoff = 1;
	yoff = 1;
	zoff = 1;
	patch = li_patch_affine(
			(PCPATCH *)patch_uncompressed,
			a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
			NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 3);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 3);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 3);
	CU_ASSERT(patch->bounds.xmin == 3);
	CU_ASSERT(patch->bounds.xmax == 3);
	CU_ASSERT(patch->bounds.ymin == 3);
	CU_ASSERT(patch->bounds.ymax == 3);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

static void
test_patch_affine_compression_none_with_dimension_names()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double a, b, c, d, e, f, g, h, i;
	double xoff, yoff, zoff;
	double v;

	// this is the same test as test_patch_affine, but explict
	// dimension names are passed to li_patch_affine

	// (1, 1, 1) is the point we're going to transform
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
	pc_pointlist_add_point(pl, pt);

	// scale + translate
	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	a = 2;
	b = 0;
	c = 0;
	d = 0;
	e = 2;
	f = 0;
	g = 0;
	h = 0;
	i = 2;
	xoff = 1;
	yoff = 1;
	zoff = 1;
	patch = li_patch_affine(
			(PCPATCH *)patch_uncompressed,
			a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
			"x", "y", "z");
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == 3);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == 3);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 3);
	CU_ASSERT(patch->bounds.xmin == 3);
	CU_ASSERT(patch->bounds.xmax == 3);
	CU_ASSERT(patch->bounds.ymin == 3);
	CU_ASSERT(patch->bounds.ymax == 3);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

#ifdef HAVE_LIBGHT
static void
test_patch_affine_compression_ght()
{
	PCPATCH *patch;
	PCPATCH_GHT *patch_ght;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double a, b, c, d, e, f, g, h, i;
	double xoff, yoff, zoff;
	double v;

	// (1, 1, 1) is the point we're going to transform
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	pc_pointlist_add_point(pl, pt);

	// scale + translate
	patch_ght = pc_patch_ght_from_pointlist(pl);
	a = 2;
	b = 0;
	c = 0;
	d = 0;
	e = 2;
	f = 0;
	g = 0;
	h = 0;
	i = 2;
	xoff = 1;
	yoff = 1;
	zoff = 1;
	patch = li_patch_affine(
			(PCPATCH *)patch_ght,
			a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
			NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 3);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 3);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == 3);
	CU_ASSERT(patch->bounds.xmin == 3);
	CU_ASSERT(patch->bounds.xmax == 3);
	CU_ASSERT(patch->bounds.ymin == 3);
	CU_ASSERT(patch->bounds.ymax == 3);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_ght);

	pc_pointlist_free(pl);
}
#endif	/* HAVE_LIBGHT */

static void
test_patch_projective_compression_none()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double v;

	// simple perspective projection which uses the origin as the center
	// of the projection, and z = 1 as the image plane

	// (1, 1, -2) is the point we're going to transform
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, -2.0);
	pc_pointlist_add_point(pl, pt);

	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	patch = li_patch_projective((PCPATCH *)patch_uncompressed,
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 1, 0,
			NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	 pc_point_get_x(pt, &v);
	CU_ASSERT(v == -0.5);
	 pc_point_get_y(pt, &v);
	CU_ASSERT(v == -0.5);
	v = pc_point_get_z(pt, &v);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == -0.5);
	CU_ASSERT(patch->bounds.xmax == -0.5);
	CU_ASSERT(patch->bounds.ymin == -0.5);
	CU_ASSERT(patch->bounds.ymax == -0.5);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

static void
test_patch_projective_compression_none_with_dimension_names()
{
	PCPATCH *patch;
	PCPATCH_UNCOMPRESSED *patch_uncompressed;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double v;

	// simple perspective projection which uses the origin as the center
	// of the projection, and z = 1 as the image plane

	// this is the same test as test_patch_projective, but explict
	// dimension names are passed to li_patch_projective

	// (1, 1, -2) is the point we're going to transform
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", -2.0);
	pc_pointlist_add_point(pl, pt);

	patch_uncompressed = pc_patch_uncompressed_from_pointlist(pl);
	patch = li_patch_projective((PCPATCH *)patch_uncompressed,
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 1, 0,
			"x", "y", "z");
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == -0.5);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == -0.5);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == -0.5);
	CU_ASSERT(patch->bounds.xmax == -0.5);
	CU_ASSERT(patch->bounds.ymin == -0.5);
	CU_ASSERT(patch->bounds.ymax == -0.5);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_uncompressed);

	pc_pointlist_free(pl);
}

#ifdef HAVE_LIBGHT
static void
test_patch_projective_compression_ght()
{
	PCPATCH *patch;
	PCPATCH_GHT *patch_ght;
	PCPOINTLIST *pl;
	PCPOINT *pt;
	double v;

	// simple perspective projection which uses the origin as the center
	// of the projection, and z = 1 as the image plane

	// (1, 1, -2) is the point we're going to transform
	pl = pc_pointlist_make(1);
	pt = pc_point_make(simpleschema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, -2.0);
	pc_pointlist_add_point(pl, pt);

	patch_ght = pc_patch_ght_from_pointlist(pl);
	patch = li_patch_projective((PCPATCH *)patch_ght,
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 1, 0,
			NULL, NULL, NULL);
	CU_ASSERT(patch != NULL);
	pt = pc_patch_pointn(patch, 1);
	CU_ASSERT(pt != NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == -0.5);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == -0.5);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == 1);
	CU_ASSERT(patch->bounds.xmin == -0.5);
	CU_ASSERT(patch->bounds.xmax == -0.5);
	CU_ASSERT(patch->bounds.ymin == -0.5);
	CU_ASSERT(patch->bounds.ymax == -0.5);
	pc_point_free(pt);
	pc_patch_free(patch);
	pc_patch_free((PCPATCH*)patch_ght);

	pc_pointlist_free(pl);
}
#endif	/* HAVE_LIBGHT */

/* REGISTER ***********************************************************/

CU_TestInfo li_patch_tests[] = {
	PC_TEST(test_patch_rotate_quaternion_compression_none),
	PC_TEST(test_patch_rotate_quaternion_compression_none_with_dimension_names),
#ifdef HAVE_LIBGHT
	PC_TEST(test_patch_rotate_quaternion_compression_ght),
#endif
	PC_TEST(test_patch_translate_compression_none),
	PC_TEST(test_patch_translate_compression_none_with_dimension_names),
#ifdef HAVE_LIBGHT
	PC_TEST(test_patch_translate_compression_ght),
#endif
	PC_TEST(test_patch_affine_compression_none),
	PC_TEST(test_patch_affine_compression_none_with_dimension_names),
#ifdef HAVE_LIBGHT
	PC_TEST(test_patch_affine_compression_ght),
#endif
	PC_TEST(test_patch_projective_compression_none),
	PC_TEST(test_patch_projective_compression_none_with_dimension_names),
#ifdef HAVE_LIBGHT
	PC_TEST(test_patch_projective_compression_ght),
#endif
	CU_TEST_INFO_NULL
};

CU_SuiteInfo li_patch_suite = {
	.pName = "li_patch",
	.pInitFunc = init_suite,
	.pCleanupFunc = clean_suite,
	.pTests = li_patch_tests
};

