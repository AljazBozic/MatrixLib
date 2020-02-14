#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "PoseOperationsShared.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// Rigid pose composition test.

	struct RigidPoseCompositionTest : testing::Test, testing::WithParamInterface<PoseCompositionData> { };

	TEST_P(RigidPoseCompositionTest, Default) {
		auto data = GetParam();

		RigidPosef pose1{ data.p1 };
		RigidPosef pose2{ data.p2 };

		common::verifyMatrix((pose1 * pose2).matrix(), data.p1 * data.p2);
	}

	INSTANTIATE_TEST_CASE_P(Default, RigidPoseCompositionTest, POSE_COMPOSITION_VALUES);

	//######################################################################################################
	// Rigid pose application test.

	struct RigidPoseApplicationTest : testing::Test, testing::WithParamInterface<PoseApplicationData> { };

	TEST_P(RigidPoseApplicationTest, Default) {
		auto data = GetParam();

		RigidPosef pose{ data.pose };
		Vec3f point{ data.point };

		common::verifyVector(pose * point, data.pose * data.point);
	}

	INSTANTIATE_TEST_CASE_P(Default, RigidPoseApplicationTest, POSE_APPLICATION_VALUES);

	//######################################################################################################
	// Rigid pose interpolation test.

	struct RigidPoseInterpolationTest : testing::Test, testing::WithParamInterface<PoseInterpolationData> { };

	TEST_P(RigidPoseInterpolationTest, Default) {
		auto data = GetParam();

		vector<RigidPosef> rigidPoses;
		for (const Mat4f& matrix : data.poses) rigidPoses.emplace_back(matrix);

		RigidPosef interpolatedPose = RigidPosef::interpolate(rigidPoses, data.weights);

		common::verifyRigidPose(interpolatedPose, RigidPosef{ data.interpolatedPose });
	}

	INSTANTIATE_TEST_CASE_P(Default, RigidPoseInterpolationTest, testing::Values(
		PoseInterpolationData{ "id x 1", { Mat4f::identity() }, { 1.f }, Mat4f::identity() },
		PoseInterpolationData{ "id x 2", { Mat4f::identity(), Mat4f::identity() }, { 0.7f, 0.3f }, Mat4f::identity() },
		PoseInterpolationData{ "translation x 1", { Mat4f::translation(1, 2, 3) }, { 1.f }, Mat4f::translation(1, 2, 3) },
		PoseInterpolationData{ "translation x 2", { Mat4f::translation(1, 2, 3), Mat4f::translation(1, -1, 1) }, { 0.5f, 0.5f }, Mat4f::translation(1, 0.5f, 2) },
		PoseInterpolationData{ "rotation x 1", { Quatf{ 0.5f, 0.3f, 0.2f, 0.4f }.matrix() }, { 1.f }, Quatf{ 0.5f, 0.3f, 0.2f, 0.4f }.matrix() },
		PoseInterpolationData{ "rotation x 2", { Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix(), Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix() }, { 0.3f, 0.7f }, Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix() }
	));

} // namespace pose_structures_tests