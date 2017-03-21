/***********************************************************************
* cu_pc_schema.c
*
*        Testing for the schema API functions
*
* Portions Copyright (c) 2012, OpenGeo
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
test_point_hex_inout()
{
	// byte:	 endianness (1 = NDR, 0 = XDR)
	// uint32:	 pcid (key to POINTCLOUD_SCHEMAS)
	// uchar[]:  pointdata (interpret relative to pcid)

	double d;
	char *hexbuf = "00000000010000000100000002000000030004";
	size_t hexsize = strlen(hexbuf);
	uint8_t *wkb = bytes_from_hexbytes(hexbuf, hexsize);
	PCPOINT *pt = pc_point_from_wkb(schema, wkb, hexsize/2);
	pc_point_get_double_by_name(pt, "X", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 0.01, 0.000001);
	pc_point_get_double_by_name(pt, "Y", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 0.02, 0.000001);
	pc_point_get_double_by_name(pt, "Z", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 0.03, 0.000001);
	pc_point_get_double_by_name(pt, "Intensity", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 4, 0.0001);
	pc_point_free(pt);
	pcfree(wkb);

	hexbuf = "01010000000100000002000000030000000500";
	hexsize = strlen(hexbuf);
	wkb = bytes_from_hexbytes(hexbuf, hexsize);
	pt = pc_point_from_wkb(schema, wkb, hexsize/2);
	pc_point_get_double_by_name(pt, "X", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 0.01, 0.000001);
	pc_point_get_double_by_name(pt, "Y", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 0.02, 0.000001);
	pc_point_get_double_by_name(pt, "Z", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 0.03, 0.000001);
	pc_point_get_double_by_name(pt, "Intensity", &d);
	CU_ASSERT_DOUBLE_EQUAL(d, 5, 0.0001);
	pc_point_free(pt);
	pcfree(wkb);

}

static void
test_point_access()
{
	PCPOINT *pt;
	double a1, a2, a3, a4, b1, b2, b3, b4;
	int idx = 0;
	double *allvals;

	pt = pc_point_make(schema);
	CU_ASSERT( pt != NULL );

	/* One at a time */
	idx = 0;
	a1 = 1.5;
	pc_point_set_double_by_index(pt, idx, a1);
	pc_point_get_double_by_index(pt, idx, &b1);
	// printf("d1=%g, d2=%g\n", a1, b1);
	CU_ASSERT_DOUBLE_EQUAL(a1, b1, 0.0000001);

	idx = 2;
	a2 = 1501500.12;
	pc_point_set_double_by_index(pt, idx, a2);
	pc_point_get_double_by_index(pt, idx, &b2);
	CU_ASSERT_DOUBLE_EQUAL(a2, b2, 0.0000001);

	a3 = 91;
	pc_point_set_double_by_name(pt, "Intensity", a3);
	pc_point_get_double_by_name(pt, "Intensity", &b3);
	CU_ASSERT_DOUBLE_EQUAL(a3, b3, 0.0000001);

	pc_point_free(pt);

	/* All at once */
	pt = pc_point_make(schema);
	a1 = 1.5;
	a2 = 1501500.12;
	a3 = 91;
	a4 = 200;
	pc_point_set_double_by_index(pt, 0, a1);
	pc_point_set_double_by_index(pt, 1, a2);
	pc_point_set_double_by_name(pt, "Intensity", a3);
	pc_point_set_double_by_name(pt, "Z", a4);
	pc_point_get_double_by_index(pt, 0, &b1);
	pc_point_get_double_by_index(pt, 1, &b2);
	pc_point_get_double_by_name(pt, "Intensity", &b3);
	pc_point_get_double_by_name(pt, "Z", &b4);
	CU_ASSERT_DOUBLE_EQUAL(a1, b1, 0.0000001);
	CU_ASSERT_DOUBLE_EQUAL(a2, b2, 0.0000001);
	CU_ASSERT_DOUBLE_EQUAL(a3, b3, 0.0000001);
	CU_ASSERT_DOUBLE_EQUAL(a4, b4, 0.0000001);

	/* as a double array */
	pc_point_set_double_by_index(pt, 0, a1);
	pc_point_set_double_by_index(pt, 1, a2);
	pc_point_set_double_by_index(pt, 2, a3);
	pc_point_set_double_by_index(pt, 3, a4);
	allvals = pc_point_to_double_array(pt);
	CU_ASSERT_DOUBLE_EQUAL(allvals[0], a1, 0.0000001);
	CU_ASSERT_DOUBLE_EQUAL(allvals[1], a2, 0.0000001);
	//printf("allvals[2]:%g\n", allvals[2]);
	CU_ASSERT_DOUBLE_EQUAL(allvals[2], a3, 0.0000001);
	//printf("allvals[3]:%g\n", allvals[3]);
	CU_ASSERT_DOUBLE_EQUAL(allvals[3], a4, 0.0000001);
	pcfree(allvals);

	pc_point_free(pt);

}

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
	pc_point_rotate_quaternion(pt, qw, qx, qy, qz, NULL, NULL, NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == -1);
	v = pc_point_get_z(pt);
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
	pc_point_rotate_quaternion(pt, qw, qx, qy, qz, NULL, NULL, NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_z(pt);
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
	pc_point_rotate_quaternion(pt, qw, qx, qy, qz, NULL, NULL, NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == -1);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 1);
	v = pc_point_get_z(pt);
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
	// dimension names are passed to pc_point_rotate_quaternion

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
	pc_point_rotate_quaternion(pt, qw, qx, qy, qz, "x", "y", "z");
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
	pc_point_translate(pt, tx, ty, tz, NULL, NULL, NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 0);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 2);
	v = pc_point_get_z(pt);
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
	// dimension names are passed to pc_point_translate

	// translation of point (1, 1, 1) by (-1, 1, 2)
	// expected result: (0, 2, 3)
	pt = pc_point_make(schema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", 1.0);
	tx = -1.0;
	ty = 1.0;
	tz = 2.0;
	pc_point_translate(pt, tx, ty, tz, "x", "y", "z");
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
	pc_point_affine(
			pt,
			a, b, c, d, e, f, g, h, i, xoff, yoff, zoff,
			NULL, NULL, NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == 3);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == 3);
	v = pc_point_get_z(pt);
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
	// dimension names are passed to pc_point_affine

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
	pc_point_affine(
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
	pc_point_projective(pt,
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 1, 0,
			NULL, NULL, NULL);
	v = pc_point_get_x(pt);
	CU_ASSERT(v == -0.5);
	v = pc_point_get_y(pt);
	CU_ASSERT(v == -0.5);
	v = pc_point_get_z(pt);
	CU_ASSERT(v == 1);
	pc_point_free(pt);
}

static void
test_point_projective_with_dimension_names()
{
	PCPOINT *pt;
	double v;

	// this is the same test as test_point_projective, but explict
	// dimension names are passed to pc_point_projective

	// simple perspective projection which uses the origin as the center
	// of the projection, and z = 1 as the image plane
	pt = pc_point_make(schema);
	pc_point_set_double_by_name(pt, "x", 1.0);
	pc_point_set_double_by_name(pt, "y", 1.0);
	pc_point_set_double_by_name(pt, "z", -2.0);
	pc_point_projective(pt,
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

CU_TestInfo point_tests[] = {
	PC_TEST(test_point_hex_inout),
	PC_TEST(test_point_access),
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

CU_SuiteInfo point_suite = {
	.pName = "point",
	.pInitFunc = init_suite,
	.pCleanupFunc = clean_suite,
	.pTests = point_tests
};
