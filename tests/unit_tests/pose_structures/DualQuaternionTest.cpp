#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "PoseOperationsShared.h"

using namespace matrix_lib;

namespace pose_structures_tests {
	
	//######################################################################################################
	// Dual quaternion interpolation test.

	struct DualQuaternionInterpolationTest : testing::Test, testing::WithParamInterface<PoseInterpolationData>{};

	TEST_P(DualQuaternionInterpolationTest, Default) {
		auto data = GetParam();
		int nPoses = data.poses.size();

		// We interpolate the pose in two ways. We want them to produce the same results.
		/// Interpolate the pose with classes.
		DualQuaternion<float> quat(Quaternion<float>(0, 0, 0, 0), Quaternion<float>(0, 0, 0, 0));

		for (int i = 0; i < nPoses; ++i) {
			Mat4f matrixPose = data.poses[i];
			float interpolationWeight = data.weights[i];

			DualQuaternion<float> dq(matrixPose.getRotation(), matrixPose.getTranslation());
			quat += DualNumber<float>(data.weights[i], 0)*dq;
		}

		Mat4f interpolatedDeformationWithClasses = quat.normalized().matrix();

		/// Interpolate the pose with functions.
		Vec8f quatVec(0.f);

		for (int i = 0; i < nPoses; ++i) {
			Mat4f matrixPose = data.poses[i];
			float interpolationWeight = data.weights[i];

			Quaternion<float> q(matrixPose.getRotation());
			Vec4f qVec(q.w(), q.x(), q.y(), q.z());
			Vec3f t(matrixPose.getTranslation());
			
			auto dualQuat = createDualQuat(qVec, t);
			auto dualNum = Vec2f(data.weights[i], 0);
			auto quatIncremented = addDualQuat(scaleDualQuat(dualNum, dualQuat), quatVec);

			quatVec[I<0>()] = quatIncremented[I<0>()];
			quatVec[I<1>()] = quatIncremented[I<1>()];
			quatVec[I<2>()] = quatIncremented[I<2>()];
			quatVec[I<3>()] = quatIncremented[I<3>()];
			quatVec[I<4>()] = quatIncremented[I<4>()];
			quatVec[I<5>()] = quatIncremented[I<5>()];
			quatVec[I<6>()] = quatIncremented[I<6>()];
			quatVec[I<7>()] = quatIncremented[I<7>()];
		}

		auto interpolatedAffinePose = convertDualQuatToAffine(quatVec);

		Mat4f interpolatedDeformationWithFunctions(
			interpolatedAffinePose[I<0>()], interpolatedAffinePose[I<1>()], interpolatedAffinePose[I<2>()], interpolatedAffinePose[I<9>()],
			interpolatedAffinePose[I<3>()], interpolatedAffinePose[I<4>()], interpolatedAffinePose[I<5>()], interpolatedAffinePose[I<10>()],
			interpolatedAffinePose[I<6>()], interpolatedAffinePose[I<7>()], interpolatedAffinePose[I<8>()], interpolatedAffinePose[I<11>()],
			0, 0, 0, 1
		);

		/// Verify that the results are the same.
		common::verifyMatrix(interpolatedDeformationWithClasses, interpolatedDeformationWithFunctions);
	}

	INSTANTIATE_TEST_CASE_P(Default, DualQuaternionInterpolationTest, testing::Values(
		PoseInterpolationData{ "id x 1",{ Mat4f::identity() },{ 1.f }, Mat4f::identity() },
		PoseInterpolationData{ "id x 2",{ Mat4f::identity(), Mat4f::identity() },{ 0.7f, 0.3f }, Mat4f::identity() },
		PoseInterpolationData{ "translation x 1",{ Mat4f::translation(1, 2, 3) },{ 1.f }, Mat4f::translation(1, 2, 3) },
		PoseInterpolationData{ "translation x 2",{ Mat4f::translation(1, 2, 3), Mat4f::translation(1, -1, 1) },{ 0.5f, 0.5f }, Mat4f::translation(1, 0.5f, 2) },
		PoseInterpolationData{ "rotation x 1",{ Quatf{ 0.5f, 0.3f, 0.2f, 0.4f }.matrix() },{ 1.f }, Quatf{ 0.5f, 0.3f, 0.2f, 0.4f }.matrix() },
		PoseInterpolationData{ "rotation x 2",{ Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix(), Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix() },{ 0.3f, 0.7f }, Quatf{ 0.3f, 0.3f, 0.3f, 0.1f }.matrix() }
	));

} // namespace pose_structures_tests