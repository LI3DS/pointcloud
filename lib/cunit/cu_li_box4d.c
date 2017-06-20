/***********************************************************************
* cu_li_box4d.c
*
***********************************************************************/

#include <math.h>
#include "CUnit/Basic.h"
#include "cu_tester.h"

/* GLOBALS ************************************************************/

/* Setup/teardown for this suite */
static int
init_suite(void)
{
	return 0;
}

static int
clean_suite(void)
{
	return 0;
}


/* TESTS **************************************************************/

static void
test_rotate_quaternion()
{
	double qw, qx, qy, qz;
	double angle;
	LIBOX4 ibox, *obox;

	// π/4 rotation around x axis
	angle = M_PI_4;
	qw = cos(angle / 2.);
	qx = sin(angle / 2.);
	qy = 0;
	qz = 0;

	// 3d box centered on (0, 0, 0)
	ibox[0][0] = -1.0;
	ibox[0][1] = -1.0;
	ibox[0][2] = 1.0;
	ibox[0][3] = 10.0;
	ibox[1][0] = 1.0;
	ibox[1][1] = 1.0;
	ibox[1][2] = -1.0;
	ibox[1][3] = 20.0;

	obox = li_box4d_rotate_quaternion(ibox, qw, qx, qy, qz);

	CU_ASSERT((*obox)[0][0] == -1.0);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[0][1], -sqrt(2.0), 0.000001);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[0][2], -sqrt(2.0), 0.000001);
	CU_ASSERT((*obox)[0][3] == 10.0);
	CU_ASSERT((*obox)[1][0] == 1.0);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[1][1], sqrt(2.0), 0.000001);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[1][2], sqrt(2.0), 0.000001);
	CU_ASSERT((*obox)[1][3] == 20.0);

	pcfree(obox);
}

static void
test_affine()
{
	double a, b, c, d, e, f, g, h, i;
	double xoff, yoff, zoff;
	double angle;
	LIBOX4 ibox, *obox;

	// create the 4x3 affine matrix
	// rotation: pi/4 along x axis
	// scale: 2 along x axis
	// translation: 1 along x axis
	angle = M_PI_4;
	a = 2;
	b = 0;
	c = 0;
	xoff = 1;
	d = 0;
	e = cos(angle);
	f = -sin(angle);
	yoff = 0;
	g = 0;
	h = sin(angle);
	i = cos(angle);
	zoff = 0;

	// 3d box centered on (0, 0, 0)
	ibox[0][0] = -1.0;
	ibox[0][1] = -1.0;
	ibox[0][2] = 1.0;
	ibox[0][3] = 10.0;
	ibox[1][0] = 1.0;
	ibox[1][1] = 1.0;
	ibox[1][2] = -1.0;
	ibox[1][3] = 20.0;

	obox = li_box4d_affine(ibox, a, b, c, d, e, f, g, h, i, xoff, yoff, zoff);

	CU_ASSERT((*obox)[0][0] == -1.0);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[0][1], -sqrt(2.0), 0.000001);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[0][2], -sqrt(2.0), 0.000001);
	CU_ASSERT((*obox)[0][3] == 10.0);
	CU_ASSERT((*obox)[1][0] == 3.0);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[1][1], sqrt(2.0), 0.000001);
	CU_ASSERT_DOUBLE_EQUAL((*obox)[1][2], sqrt(2.0), 0.000001);
	CU_ASSERT((*obox)[1][3] == 20.0);

	pcfree(obox);
}

/* REGISTER ***********************************************************/

CU_TestInfo li_box4d_tests[] = {
	PC_TEST(test_rotate_quaternion),
	PC_TEST(test_affine),
	CU_TEST_INFO_NULL
};

CU_SuiteInfo li_box4d_suite = {
	.pName = "li_box4d",
	.pInitFunc = init_suite,
	.pCleanupFunc = clean_suite,
	.pTests = li_box4d_tests
};
