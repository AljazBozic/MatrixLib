#include <gtest/gtest.h>

#include "PoseOperationsShared.h"
#include "matrix_lib/MatrixLibInclude.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// Affine pose increment application test.

	struct AffinePoseIncrementApplicationTest : testing::Test, testing::WithParamInterface<PoseApplicationData> { };

	TEST_P(AffinePoseIncrementApplicationTest, Default) {
		auto data = GetParam();

		AffineIncrementd pose{ data.pose };
		Vec3d point{ data.point };

		common::verifyVector(Vec3f{ pose * point }, data.pose * data.point);
	}

	INSTANTIATE_TEST_CASE_P(Default, AffinePoseIncrementApplicationTest, POSE_APPLICATION_VALUES);

	//######################################################################################################
	// Affine pose increment interpolation test.

	struct AffinePoseIncrementInterpolationTest : testing::Test, testing::WithParamInterface<PoseInterpolationData> {};

	TEST_P(AffinePoseIncrementInterpolationTest, Default) {
		auto data = GetParam();

		vector<AffineIncrementf> affinePoses;
		for (const Mat4f& matrix : data.poses) affinePoses.emplace_back(matrix);

		AffineIncrementf interpolatedPose = AffineIncrementf::interpolate(affinePoses, data.weights);

		common::verifyMatrix(interpolatedPose.getAffineMatrix(), data.interpolatedPose.getRotation());
		common::verifyVector(interpolatedPose.getTranslation(), data.interpolatedPose.getTranslation());
	}

	INSTANTIATE_TEST_CASE_P(Default, AffinePoseIncrementInterpolationTest, testing::Values(
		PoseInterpolationData{ "id x 1",{ Mat4f::identity() },{ 1.f }, Mat4f::identity() },
		PoseInterpolationData{ "id x 2",{ Mat4f::identity(), Mat4f::identity() },{ 0.7f, 0.3f }, Mat4f::identity() },
		PoseInterpolationData{ "translation x 1",{ Mat4f::translation(1, 2, 3) },{ 1.f }, Mat4f::translation(1, 2, 3) },
		PoseInterpolationData{ "translation x 2",{ Mat4f::translation(1, 2, 3), Mat4f::translation(1, -1, 1) },{ 0.5f, 0.5f }, Mat4f::translation(1, 0.5f, 2) },
		PoseInterpolationData{ "rotation x 1",{ Mat4f::rotation(20, 30, 40) },{ 1.f }, Mat4f::rotation(20, 30, 40) },
		PoseInterpolationData{ "rotation x 2",{ Mat4f::rotation(20, 30, 40), Mat4f::rotation(10, 20, 30) },{ 0.3f, 0.7f }, 0.3f * Mat4f::rotation(20, 30, 40) + 0.7f * Mat4f::rotation(10, 20, 30) },
		PoseInterpolationData{ "pose x 1",{ Mat4f::pose(20, 30, 40, 1, 2, 3) },{ 1.f }, Mat4f::pose(20, 30, 40, 1, 2, 3) },
		PoseInterpolationData{ "pose x 2",{ Mat4f::pose(20, 30, 40, 1, 2, 3), Mat4f::pose(10, 20, 30, 1, -2, -5) },{ 0.3f, 0.7f }, 0.3f * Mat4f::pose(20, 30, 40, 1, 2, 3) + 0.7f * Mat4f::pose(10, 20, 30, 1, -2, -5) }
	));

} // namespace pose_structures_tests