#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// UnitQuaternion matrix <-> quaternion conversion test.

	struct MatToUnitQuaternionConversionTest : testing::Test, testing::WithParamInterface<Mat3f> { };

	TEST_P(MatToUnitQuaternionConversionTest, Default) {
		const Mat3f rotation = GetParam();

		common::verifyMatrix(Quatf{ rotation }.matrix(), rotation);
	}

	INSTANTIATE_TEST_CASE_P(Default, MatToUnitQuaternionConversionTest, testing::Values(
		Mat3f::identity(),
		Mat3f::rotation(10, 20, 30),
		Mat3f::rotation(-10, -20, -30),
		Mat3f::rotation(1, 2, 3)
	));

	struct UnitQuaternionToMatConversionTest : testing::Test, testing::WithParamInterface<Quatf> { };

	TEST_P(UnitQuaternionToMatConversionTest, Default) {
		const Quatf rotation = GetParam();

		common::verifyVector(Quatf{ rotation.matrix() }.imag(), rotation.imag());
		EXPECT_FLOAT_EQ(Quatf{ rotation.matrix() }.real(), rotation.real());
	}

	INSTANTIATE_TEST_CASE_P(Default, UnitQuaternionToMatConversionTest, testing::Values(
		Quatf{ 1, 0, 0, 0 },
		Quatf{ 0.5f, 0.3f, 0.2f, 0.4f },
		Quatf{ 0.5f, -0.3f, -0.2f, 0.4f },
		Quatf{ 0.2f, -0.1f, -0.1f, 0.1f }
	));

} // namespace pose_structures_tests