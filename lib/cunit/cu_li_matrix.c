
/***********************************************************************
* cu_li_matrix.c
*
*        Testing for the matrix functions
*
* Copyright (c) 2017, Mathieu Brédif, IGN
*
***********************************************************************/

#include "CUnit/Basic.h"
#include "cu_tester.h"

/* GLOBALS ************************************************************/

#define N (5)
const LIMAT44 mat[N] = {
	{ 1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 },
	{ 0,0,0,1,  0,0,1,0,  0,1,0,0,  1,0,0,0 },
	{ 1,2,3,4,  5,6,7,8,  9,10,11,12,  13,14,15,16 },
	{ 1,1,1,1,  2,4,8,16,  3,9,27,81, 4,16,64,512 },
	{ 0,1,0,0,  1,0,0,0,  0,0,1,0,  0,0,0,1 }
};

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
test_matrix_determinant()
{
	int i;
	double expected[] = { 1,1,0,3360,-1 };
	for( i = 0; i < N; ++i )
		CU_ASSERT_DOUBLE_EQUAL(li_matrix_44_determinant(mat[i]),expected[i], 0.1);
}

static void
test_matrix_adjugate()
{
	LIMAT44 adj;
	const LIMAT44 expected[N] = {
		{ 1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 },
		{ 0,0,0,1,  0,0,1,0,  0,1,0,0,  1,0,0,0 },
		{ 0,0,0,0,  0,0,0,0,  0,0,0,0,  0,0,0,0 },
		{ 10368,-5472,1408,-72,  - 8928,7512,-2208,132,  1968,-2112,848,-72, -48,72,-48,12 },
		{ 0,-1,0,0,  -1,0,0,0,  0,0,-1,0,  0,0,0,-1 }
	};
	int i,j;
	for( i = 0; i < N; ++i )
	{
		li_matrix_44_adjugate(adj,mat[i],NULL);
		for( j = 0; j < 16; ++j )
			CU_ASSERT_DOUBLE_EQUAL(adj[j],expected[i][j],0.000001);
	}
}

static void
test_matrix_inverse()
{
	LIMAT44 inv, mul;
	int inversible[] = { 1,1,0,1,1 };
	int i, j, rv;
	for( i = 0; i < N; ++i )
	{
		rv = li_matrix_44_inverse(inv,mat[i]);
		CU_ASSERT(rv == inversible[i]);
		if(!rv) continue;
		li_matrix_44_multiply_matrix_44(mul,inv,mat[i]);
		for( j = 0; j < 16; ++j )
			CU_ASSERT_DOUBLE_EQUAL(mul[j],((j%5)==0)?1:0,0.000001);
	}
}

/* REGISTER ***********************************************************/

CU_TestInfo li_matrix_tests[] = {
	PC_TEST(test_matrix_determinant),
	PC_TEST(test_matrix_adjugate),
	PC_TEST(test_matrix_inverse),
	CU_TEST_INFO_NULL
};

CU_SuiteInfo li_matrix_suite = {
	.pName = "li_matrix",
	.pInitFunc = init_suite,
	.pCleanupFunc = clean_suite,
	.pTests = li_matrix_tests
};
