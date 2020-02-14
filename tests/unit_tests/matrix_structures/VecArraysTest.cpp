#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;

namespace matrix_structures_tests {

	TEST(VecArraysTest, AVecX_CompileTimeIndexingBaseType) {
		AVecX<false, float, 5> sAVec;
		sAVec.allocate(3);

		AVecX<true, float, 5> iAVec;
		iAVec.wrap(sAVec.getPointerList());

		for (int i = 0; i < 3; ++i) {
			iAVec[I<0>()][i] = i * 1.f;
			iAVec[I<1>()][i] = i * 2.f;
			iAVec[I<2>()][i] = i * 3.f;
			iAVec[I<3>()][i] = i * 4.f;
			iAVec[I<4>()][i] = i * 5.f;
		}

		for (int i = 0; i < 3; ++i) {
			EXPECT_FLOAT_EQ(iAVec[I<0>()][i], i * 1.f);
			EXPECT_FLOAT_EQ(iAVec[I<1>()][i], i * 2.f);
			EXPECT_FLOAT_EQ(iAVec[I<2>()][i], i * 3.f);
			EXPECT_FLOAT_EQ(iAVec[I<3>()][i], i * 4.f);
			EXPECT_FLOAT_EQ(iAVec[I<4>()][i], i * 5.f);
		}
	}

	TEST(VecArraysTest, AVecX_RunTimeIndexingBaseType) {
		AVecX<false, float, 5> sAVec;
		sAVec.allocate(3);

		AVecX<true, float, 5> iAVec;
		iAVec.wrap(sAVec.getPointerList());

		for (int i = 0; i < 3; ++i) {
			iAVec[0][i] = i * 1.f;
			iAVec[1][i] = i * 2.f;
			iAVec[2][i] = i * 3.f;
			iAVec[3][i] = i * 4.f;
			iAVec[4][i] = i * 5.f;
		}

		for (int i = 0; i < 3; ++i) {
			EXPECT_FLOAT_EQ(iAVec[0][i], i * 1.f);
			EXPECT_FLOAT_EQ(iAVec[1][i], i * 2.f);
			EXPECT_FLOAT_EQ(iAVec[2][i], i * 3.f);
			EXPECT_FLOAT_EQ(iAVec[3][i], i * 4.f);
			EXPECT_FLOAT_EQ(iAVec[4][i], i * 5.f);
		}
	}

	TEST(VecArraysTest, AVecX_CompileTimeIndexingSoAType) {
		AVecX<false, sAMat3f, 5> sAVec;
		sAVec.allocate(3);

		AVecX<true, iAMat3f, 5> iAVec;
		iAVec.wrap(sAVec.getPointerList());

		for (int i = 0; i < 3; ++i) {
			Mat3f m{ i * 1.f, i * 2.f, i * 3.f, i * 4.f, i * 5.f, i * 6.f, i * 7.f, i * 8.f, i * 9.f };
			soa::store(m * 1.f, iAVec[I<0>()], i);
			soa::store(m * 2.f, iAVec[I<1>()], i);
			soa::store(m * 3.f, iAVec[I<2>()], i);
			soa::store(m * 4.f, iAVec[I<3>()], i);
			soa::store(m * 5.f, iAVec[I<4>()], i);
		}

		for (int i = 0; i < 3; ++i) {
			Mat3f m{ i * 1.f, i * 2.f, i * 3.f, i * 4.f, i * 5.f, i * 6.f, i * 7.f, i * 8.f, i * 9.f };
			common::verifyMatrix(soa::load(iAVec[I<0>()], i), m * 1.f);
			common::verifyMatrix(soa::load(iAVec[I<1>()], i), m * 2.f);
			common::verifyMatrix(soa::load(iAVec[I<2>()], i), m * 3.f);
			common::verifyMatrix(soa::load(iAVec[I<3>()], i), m * 4.f);
			common::verifyMatrix(soa::load(iAVec[I<4>()], i), m * 5.f);
		}
	}

	TEST(VecArraysTest, AVecX_RunTimeIndexingSoAType) {
		AVecX<false, sAMat3f, 5> sAVec;
		sAVec.allocate(3);

		AVecX<true, iAMat3f, 5> iAVec;
		iAVec.wrap(sAVec.getPointerList());

		for (int i = 0; i < 3; ++i) {
			Mat3f m{ i * 1.f, i * 2.f, i * 3.f, i * 4.f, i * 5.f, i * 6.f, i * 7.f, i * 8.f, i * 9.f };
			soa::store(m * 1.f, iAVec[0], i);
			soa::store(m * 2.f, iAVec[1], i);
			soa::store(m * 3.f, iAVec[2], i);
			soa::store(m * 4.f, iAVec[3], i);
			soa::store(m * 5.f, iAVec[4], i);
		}

		for (int i = 0; i < 3; ++i) {
			Mat3f m{ i * 1.f, i * 2.f, i * 3.f, i * 4.f, i * 5.f, i * 6.f, i * 7.f, i * 8.f, i * 9.f };
			common::verifyMatrix(soa::load(iAVec[0], i), m * 1.f);
			common::verifyMatrix(soa::load(iAVec[1], i), m * 2.f);
			common::verifyMatrix(soa::load(iAVec[2], i), m * 3.f);
			common::verifyMatrix(soa::load(iAVec[3], i), m * 4.f);
			common::verifyMatrix(soa::load(iAVec[4], i), m * 5.f);
		}
	}

} // namespace matrix_structures_tests