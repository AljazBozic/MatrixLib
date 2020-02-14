#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "PoseOperationsShared.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// Affine pose composition test.

	struct AffinePoseCompositionTest : testing::Test, testing::WithParamInterface<PoseCompositionData> { };

	TEST_P(AffinePoseCompositionTest, Default) {
		auto data = GetParam();

		AffinePosef pose1{ data.p1 };
		AffinePosef pose2{ data.p2 };

		common::verifyMatrix((pose1 * pose2).matrix(), data.p1 * data.p2);
	}

	INSTANTIATE_TEST_CASE_P(Default, AffinePoseCompositionTest, POSE_COMPOSITION_VALUES);

	//######################################################################################################
	// Affine pose application test.

	struct AffinePoseApplicationTest : testing::Test, testing::WithParamInterface<PoseApplicationData> { };

	TEST_P(AffinePoseApplicationTest, Default) {
		auto data = GetParam();

		AffinePosef pose{ data.pose };
		Vec3f point{ data.point };

		common::verifyVector(pose * point, data.pose * data.point);
	}

	INSTANTIATE_TEST_CASE_P(Default, AffinePoseApplicationTest, POSE_APPLICATION_VALUES);

	//######################################################################################################
	// Affine pose interpolation test.

	struct AffinePoseInterpolationTest : testing::Test, testing::WithParamInterface<PoseInterpolationData> { };

	TEST_P(AffinePoseInterpolationTest, Default) {
		auto data = GetParam();

		vector<AffinePosef> affinePoses;
		for (const Mat4f& matrix : data.poses) affinePoses.emplace_back(matrix);
		
		AffinePosef interpolatedPose = AffinePosef::interpolate(affinePoses, data.weights);

		common::verifyAffinePose(interpolatedPose, AffinePosef{ data.interpolatedPose });
	}

	INSTANTIATE_TEST_CASE_P(Default, AffinePoseInterpolationTest, testing::Values(
		PoseInterpolationData{ "id x 1", { Mat4f::identity() }, { 1.f }, Mat4f::identity() },
		PoseInterpolationData{ "id x 2", { Mat4f::identity(), Mat4f::identity() }, { 0.7f, 0.3f }, Mat4f::identity() },
		PoseInterpolationData{ "translation x 1", { Mat4f::translation(1, 2, 3) }, { 1.f }, Mat4f::translation(1, 2, 3) },
		PoseInterpolationData{ "translation x 2", { Mat4f::translation(1, 2, 3), Mat4f::translation(1, -1, 1) }, { 0.5f, 0.5f }, Mat4f::translation(1, 0.5f, 2) },
		PoseInterpolationData{ "rotation x 1", { Mat4f::rotation(20, 30, 40) }, { 1.f }, Mat4f::rotation(20, 30, 40) },
		PoseInterpolationData{ "rotation x 2", { Mat4f::rotation(20, 30, 40), Mat4f::rotation(10, 20, 30) }, { 0.3f, 0.7f }, 0.3f * Mat4f::rotation(20, 30, 40) + 0.7f * Mat4f::rotation(10, 20, 30) },
		PoseInterpolationData{ "pose x 1", { Mat4f::pose(20, 30, 40, 1, 2, 3) }, { 1.f }, Mat4f::pose(20, 30, 40, 1, 2, 3) },
		PoseInterpolationData{ "pose x 2", { Mat4f::pose(20, 30, 40, 1, 2, 3), Mat4f::pose(10, 20, 30, 1, -2, -5) }, { 0.3f, 0.7f }, 0.3f * Mat4f::pose(20, 30, 40, 1, 2, 3) + 0.7f * Mat4f::pose(10, 20, 30, 1, -2, -5) }
	));

} // namespace pose_structures_tests