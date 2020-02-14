#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// SO3 matrix <-> SO3 conversion test.

	struct MatToSO3ConversionTest : testing::Test, testing::WithParamInterface<Mat3f> { };

	TEST_P(MatToSO3ConversionTest, Default) {
		const Mat3f rotation = GetParam();

		common::verifyMatrix(SO3f{ rotation }.matrix(), rotation);
	}

	INSTANTIATE_TEST_CASE_P(Default, MatToSO3ConversionTest, testing::Values(
		Mat3f::identity(),
		Mat3f::rotation(10, 20, 30),
		Mat3f::rotation(-10, -20, -30),
		Mat3f::rotation(1, 2, 3)
	));

	struct SO3ToMatConversionTest : testing::Test, testing::WithParamInterface<SO3f> { };

	TEST_P(SO3ToMatConversionTest, Default) {
		const SO3f rotation = GetParam();
		
		common::verifyVector(SO3f{ rotation.matrix() }.getOmega(), rotation.getOmega());
	}

	INSTANTIATE_TEST_CASE_P(Default, SO3ToMatConversionTest, testing::Values(
		SO3f{ 0, 0, 0 },
		SO3f{ 1, 2, 0 },
		SO3f{ -0.5f, 1.f, 2.f },
		SO3f{ 2.f, -1.f, 0.5f }
	));

} // namespace pose_structures_tests