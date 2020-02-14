#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "PoseOperationsShared.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// SO3wT application test.

	struct SO3wTApplicationTest : testing::Test, testing::WithParamInterface<PoseApplicationData> { };

	TEST_P(SO3wTApplicationTest, Default) {
		auto data = GetParam();

		SO3wTd pose{ data.pose };
		Vec3d point{ data.point };

		common::verifyVector(Vec3f{ pose * point }, data.pose * data.point);
	}

	INSTANTIATE_TEST_CASE_P(Default, SO3wTApplicationTest, POSE_APPLICATION_VALUES);

	//######################################################################################################
	// SO3wT interpolation test.

	struct SO3wTInterpolationTest : testing::Test, testing::WithParamInterface<PoseInterpolationData> {};

	TEST_P(SO3wTInterpolationTest, Default) {
		auto data = GetParam();

		vector<SO3wTf> SO3wTPoses;
		for (const Mat4f& matrix : data.poses) SO3wTPoses.emplace_back(matrix);

		SO3wTf interpolatedPose = SO3wTf::interpolate(SO3wTPoses, data.weights);

		common::verifyMatrix(interpolatedPose.getAxisAngle().matrix(), data.interpolatedPose.getRotation());
		common::verifyVector(interpolatedPose.getTranslation(), data.interpolatedPose.getTranslation());
	}

	INSTANTIATE_TEST_CASE_P(Default, SO3wTInterpolationTest, testing::Values(
		PoseInterpolationData{ "id x 1",{ Mat4f::identity() },{ 1.f }, Mat4f::identity() },
		PoseInterpolationData{ "id x 2",{ Mat4f::identity(), Mat4f::identity() },{ 0.7f, 0.3f }, Mat4f::identity() },
		PoseInterpolationData{ "translation x 1",{ Mat4f::translation(1, 2, 3) },{ 1.f }, Mat4f::translation(1, 2, 3) },
		PoseInterpolationData{ "translation x 2",{ Mat4f::translation(1, 2, 3), Mat4f::translation(1, -1, 1) },{ 0.5f, 0.5f }, Mat4f::translation(1, 0.5f, 2) },
		PoseInterpolationData{ "rotation x 1",{ Quatf{ 0.5f, 0.3f, 0.2f, 0.4f }.matrix() },{ 1.f }, Quatf{ 0.5f, 0.3f, 0.2f, 0.4f }.matrix() },
		PoseInterpolationData{ "rotation x 2",{ Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix(), Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix() },{ 0.3f, 0.7f }, Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix() }
	));

} // namespace pose_structures_tests