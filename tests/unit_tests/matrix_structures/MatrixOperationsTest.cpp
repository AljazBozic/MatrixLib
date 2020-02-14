#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;

namespace matrix_structures_tests {

	TEST(MatrixOperationsTest, Invert_Mat2) {
		Mat2f m{ 1.f, 2.f, 3.f, 4.f };
		Mat2f mInv = Mat2f{ invert2x2(m) };

		common::verifyMatrix(m * mInv, Mat2f::identity());
		common::verifyMatrix(mInv * m, Mat2f::identity());
	}

	TEST(MatrixOperationsTest, Invert_Mat3) {
		Mat3f m{ 1.f, 2.f, 3.f, 0.f, 1.f, 2.f, 9.f, 0.f, 1.f };
		Mat3f mInv = Mat3f{ invert3x3(m) };

		common::verifyMatrix(m * mInv, Mat3f::identity());
		common::verifyMatrix(mInv * m, Mat3f::identity());
	}

	TEST(MatrixOperationsTest, Invert_Mat4) {
		Mat4f m{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 0.f, 11.f, 12.f, 13.f, 14.f, 0.f, 16.f };
		Mat4f mInv = Mat4f{ invert4x4(m) };

		common::verifyMatrix(m * mInv, Mat4f::identity());
		common::verifyMatrix(mInv * m, Mat4f::identity());
	}

} // namespace matrix_structures_tests