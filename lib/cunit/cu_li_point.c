/***********************************************************************
* cu_li_point.c
*
***********************************************************************/

#include "CUnit/Basic.h"
#include "cu_tester.h"

/* GLOBALS ************************************************************/

static PCSCHEMA *schema = NULL;
static const char *xmlfile = "data/simple-schema.xml";

// SIMPLE SCHEMA
// int32_t x
// int32_t y
// int32_t z
// int16_t intensity

/* Setup/teardown for this suite */
static int
init_suite(void)
{
	char *xmlstr = file_to_str(xmlfile);
	schema = pc_schema_from_xml(xmlstr);
	pcfree(xmlstr);
	if ( !schema ) return 1;
	return 0;
}

static int
clean_suite(void)
{
	pc_schema_free(schema);
	return 0;
}


/* TESTS **************************************************************/

static void
test_point_rotate_quaternion()
{
	PCPOINT *pt;
	double angle;
	double qw, qx, qy, qz;
	double v;

	// π/2 rotation of point (1, 1, 1) around x axis
	// expected result: (1, -1, 1)
	pt = pc_point_make(schema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = sin(angle / 2.);
	qy = 0;
	qz = 0;
	li_point_rotate_quaternion(pt, qw, qx, qy, qz, "", "", "");
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == -1);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_free(pt);

	// π/2 rotation of point (1, 1, 1) around y axis
	// expected result: (1, 1, -1)
	pt = pc_point_make(schema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = sin(angle / 2.);
	qz = 0;
	li_point_rotate_quaternion(pt, qw, qx, qy, qz, "", "", "");
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == -1);
	pc_point_free(pt);

	// π/2 rotation of point (1, 1, 1) around z axis
	// expected result: (-1, 1, 1)
	pt = pc_point_make(schema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = 0;
	qz = sin(angle / 2.);
	li_point_rotate_quaternion(pt, qw, qx, qy, qz, "", "", "");
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == -1);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_free(pt);
}

static void
test_point_rotate_quaternion_with_dimension_names()
{
	PCPOINT *pt;
	double angle;
	double qw, qx, qy, qz;
	double v;

	// this is the same test as test_point_rotate_quaternion, but explict
	// dimension names are passed to li_point_rotate_quaternion

	// π/2 rotation of point (1, 1, 1) around z axis
	// expected result: (-1, 1, 1)
	pt = pc_point_make(schema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
	angle = M_PI_2;
	qw = cos(angle / 2.);
	qx = 0;
	qy = 0;
	qz = sin(angle / 2.);
	li_point_rotate_quaternion(pt, qw, qx, qy, qz, "x", "y", "z");
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == -1);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == 1);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 1);
	pc_point_free(pt);
}

static void
test_point_translate()
{
	PCPOINT *pt;
	double tx, ty, tz;
	double v;

	// translation of point (1, 1, 1) by (-1, 1, 2)
	// expected result: (0, 2, 3)
	pt = pc_point_make(schema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
	tx = -1.0;
	ty = 1.0;
	tz = 2.0;
	li_point_translate(pt, tx, ty, tz, "", "", "");
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 0);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 2);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 3);
	pc_point_free(pt);
}

static void
test_point_translate_with_dimension_names()
{
	PCPOINT *pt;
	double tx, ty, tz;
	double v;

	// this is the same test as test_point_translate, but explict
	// dimension names are passed to li_point_translate

	// translation of point (1, 1, 1) by (-1, 1, 2)
	// expected result: (0, 2, 3)
	pt = pc_point_make(schema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
	tx = -1.0;
	ty = 1.0;
	tz = 2.0;
	li_point_translate(pt, tx, ty, tz, "x", "y", "z");
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == 0);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == 2);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 3);
	pc_point_free(pt);
}

static void
test_point_affine()
{
	PCPOINT *pt;
	double a, b, c, d, e, f, g, h, i;
	double xoff, yoff, zoff;
	double v;

	// scale + translate
	pt = pc_point_make(schema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, 1.0);
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
	li_point_affine(
			pt,
			a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
			"", "", "");
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == 3);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == 3);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 3);
	pc_point_free(pt);
}

static void
test_point_affine_with_dimension_names()
{
	PCPOINT *pt;
	double a, b, c, d, e, f, g, h, i;
	double xoff, yoff, zoff;
	double v;

	// this is the same test as test_point_affine, but explict
	// dimension names are passed to li_point_affine

	// scale + translate
	pt = pc_point_make(schema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
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
	li_point_affine(
			pt,
			a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
			"x", "y", "z");
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == 3);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == 3);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 3);
	pc_point_free(pt);
}

static void
test_point_projective()
{
	PCPOINT *pt;
	double v;

	// simple perspective projection which uses the origin as the center
	// of the projection, and z = 1 as the image plane
	pt = pc_point_make(schema);
	pc_point_set_x(pt, 1.0);
	pc_point_set_y(pt, 1.0);
	pc_point_set_z(pt, -2.0);
	li_point_projective(pt,
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 1, 0,
			"", "", "");
	pc_point_get_x(pt, &v);
	CU_ASSERT(v == -0.5);
	pc_point_get_y(pt, &v);
	CU_ASSERT(v == -0.5);
	pc_point_get_z(pt, &v);
	CU_ASSERT(v == 1);
	pc_point_free(pt);
}

static void
test_point_projective_with_dimension_names()
{
	PCPOINT *pt;
	double v;

	// this is the same test as test_point_projective, but explict
	// dimension names are passed to li_point_projective

	// simple perspective projection which uses the origin as the center
	// of the projection, and z = 1 as the image plane
	pt = pc_point_make(schema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", -2.0);
	li_point_projective(pt,
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 1, 0,
			"x", "y", "z");
	pc_point_get_double_by_name(pt, "x", &v);
	CU_ASSERT(v == -0.5);
	pc_point_get_double_by_name(pt, "y", &v);
	CU_ASSERT(v == -0.5);
	pc_point_get_double_by_name(pt, "z", &v);
	CU_ASSERT(v == 1);
	pc_point_free(pt);
}

/* REGISTER ***********************************************************/

CU_TestInfo li_point_tests[] = {
	PC_TEST(test_point_rotate_quaternion),
	PC_TEST(test_point_rotate_quaternion_with_dimension_names),
	PC_TEST(test_point_translate),
	PC_TEST(test_point_translate_with_dimension_names),
	PC_TEST(test_point_affine),
	PC_TEST(test_point_affine_with_dimension_names),
	PC_TEST(test_point_projective),
	PC_TEST(test_point_projective_with_dimension_names),
	CU_TEST_INFO_NULL
};

CU_SuiteInfo li_point_suite = {
	.pName = "li_point",
	.pInitFunc = init_suite,
	.pCleanupFunc = clean_suite,
	.pTests = li_point_tests
};
