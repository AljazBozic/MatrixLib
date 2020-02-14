#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"
#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	//######################################################################################################
	// Pose structures test.
	
	TEST(PoseOperationsTest, RotatePointWithSO3_Zero) {
		const Vec3f so3{ 0, 0, 0 };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithSO3(so3, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ 1, 2, 3 });
	}

	TEST(PoseOperationsTest, RotatePointWithSO3_FullAxisX) {
		const Vec3f so3{ math_proc::PIf, 0, 0 };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithSO3(so3, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ 1, -2, -3 });
	}

	TEST(PoseOperationsTest, RotatePointWithSO3_HalfAxisX) {
		const Vec3f so3{ math_proc::PIf/2, 0, 0 };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithSO3(so3, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ 1, -3, 2 });
	}

	TEST(PoseOperationsTest, RotatePointWithSO3_MinusHalfAxisY) {
		const Vec3f so3{ 0, -math_proc::PIf/2, 0 };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithSO3(so3, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ -3, 2, 1 });
	}

	TEST(PoseOperationsTest, RotatePointWithUnitQuaternion_Zero) {
		const Vec4f quat{ 1, 0, 0, 0 };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithUnitQuaternion(quat, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ 1, 2, 3 });
	}

	TEST(PoseOperationsTest, RotatePointWithUnitQuaternion_FullAxisX) {
		const Vec3f axis{ 1, 0, 0 };
		const float angle{ math_proc::PIf };
		const Vec4f quat{ cos(angle/2.f), sin(angle/2.f) * axis.x(), sin(angle / 2.f) * axis.y(), sin(angle / 2.f) * axis.z() };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithUnitQuaternion(quat, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ 1, -2, -3 });
	}

	TEST(PoseOperationsTest, RotatePointWithUnitQuaternion_HalfAxisX) {
		const Vec3f axis{ 1, 0, 0 };
		const float angle{ math_proc::PIf / 2 };
		const Vec4f quat{ cos(angle / 2.f), sin(angle / 2.f) * axis.x(), sin(angle / 2.f) * axis.y(), sin(angle / 2.f) * axis.z() };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithUnitQuaternion(quat, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ 1, -3, 2 });
	}

	TEST(PoseOperationsTest, RotatePointWithUnitQuaternion_MinusHalfAxisY) {
		const Vec3f axis{ 0, 1, 0 };
		const float angle{ -math_proc::PIf / 2 };
		const Vec4f quat{ cos(angle / 2.f), sin(angle / 2.f) * axis.x(), sin(angle / 2.f) * axis.y(), sin(angle / 2.f) * axis.z() };
		const Vec3f point{ 1, 2, 3 };

		auto rotatedPoint = rotatePointWithUnitQuaternion(quat, point);

		common::verifyVector(Vec3f{ rotatedPoint }, Vec3f{ -3, 2, 1 });
	}

	TEST(PoseOperationsTest, RigidIncrementWrapper) {
		double twistData[] = { 1, 0, 0, 1, 0, 1 };
		SO3wTd twistIncrement(twistData);

		common::verifyVector(twistIncrement.getAxisAngle().getOmega(), Vec3d{ 1, 0, 0 });
		common::verifyVector(twistIncrement.getTranslation(), Vec3d{ 1, 0, 1 });
	}

	TEST(PoseOperationsTest, AffineIncrementWrapper) {
		double affinePoseData[] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1 };
		AffineIncrementd affineIncrement{ affinePoseData };

		common::verifyMatrix(affineIncrement.getAffineMatrix(), Mat3d::identity());
		common::verifyVector(affineIncrement.getTranslation(), Vec3d{ 1, 1, 1 });
	}

	TEST(PoseOperationsTest, RigidPoseSize) {
		RigidPose<double> poseDouble;
		EXPECT_EQ(sizeof(poseDouble), 7 * sizeof(double));

		RigidPose<float> poseFloat;
		EXPECT_EQ(sizeof(poseFloat), 7 * sizeof(float));
	}

	TEST(PoseOperationsTest, AffinePoseSize) {
		AffinePose<double> poseDouble;
		EXPECT_EQ(sizeof(poseDouble), 12 * sizeof(double));

		AffinePose<float> poseFloat;
		EXPECT_EQ(sizeof(poseFloat), 12 * sizeof(float));
	}

	TEST(PoseOperationsTest, RigidIncrementSize) {
		SO3wT<double> poseIncrementDouble;
		EXPECT_EQ(sizeof(poseIncrementDouble), 6 * sizeof(double));

		SO3wT<float> poseIncrementFloat;
		EXPECT_EQ(sizeof(poseIncrementFloat), 6 * sizeof(float));
	}

	TEST(PoseOperationsTest, AffineIncrementSize) {
		AffineIncrement<double> poseIncrementDouble;
		EXPECT_EQ(sizeof(poseIncrementDouble), 12 * sizeof(double));

		AffineIncrement<float> poseIncrementFloat;
		EXPECT_EQ(sizeof(poseIncrementFloat), 12 * sizeof(float));
	}


	//######################################################################################################
	// Conversion of rotation vector to matrix test.
	
	struct ConvertUnitQuaternionToMatTest : testing::Test, testing::WithParamInterface<Mat3f> { };

	TEST_P(ConvertUnitQuaternionToMatTest, Default) {
		const Mat3f rotationMatrix = GetParam();
		const Quatf quaternion{ rotationMatrix };
		auto computedRotationMatrix = convertUnitQuaternionToMatrix(quaternion);

		common::verifyMatrix(Mat3f{ computedRotationMatrix }, rotationMatrix);
	}

	INSTANTIATE_TEST_CASE_P(Default, ConvertUnitQuaternionToMatTest, testing::Values(
		Mat3f::identity(),
		Mat3f::rotation(10, 20, 30),
		Mat3f::rotation(-10, -20, -30),
		Mat3f::rotation(1, 2, 3)
	));

	struct ConvertSO3ToMatTest : testing::Test, testing::WithParamInterface<Mat3f> { };

	TEST_P(ConvertSO3ToMatTest, Default) {
		const Mat3f rotationMatrix = GetParam();
		const SO3f omega{ rotationMatrix };
		auto computedRotationMatrix = convertSO3ToMatrix(omega);

		common::verifyMatrix(Mat3f{ computedRotationMatrix }, rotationMatrix);
	}

	INSTANTIATE_TEST_CASE_P(Default, ConvertSO3ToMatTest, testing::Values(
		Mat3f::identity(),
		Mat3f::rotation(10, 20, 30),
		Mat3f::rotation(-10, -20, -30),
		Mat3f::rotation(1, 2, 3)
	));


	//######################################################################################################
	// Conversion of matrix to rotation vector test.

	struct ConvertMatToUnitQuaternionTest : testing::Test, testing::WithParamInterface<Mat3f> {};

	TEST_P(ConvertMatToUnitQuaternionTest, Default) {
		const Mat3f rotationMatrix = GetParam();
		auto computedRotationMatrix = convertUnitQuaternionToMatrix(convertMatrixToUnitQuaternion(rotationMatrix));

		common::verifyMatrix(Mat3f{ computedRotationMatrix }, rotationMatrix);
	}

	INSTANTIATE_TEST_CASE_P(Default, ConvertMatToUnitQuaternionTest, testing::Values(
		Mat3f::identity(),
		Mat3f::rotation(10, 20, 30),
		Mat3f::rotation(-10, -20, -30),
		Mat3f::rotation(1, 2, 3)
	));

	struct ConvertMatToSO3Test : testing::Test, testing::WithParamInterface<Mat3f> {};

	TEST_P(ConvertMatToSO3Test, Default) {
		const Mat3f rotationMatrix = GetParam();
		auto computedRotationMatrix = convertSO3ToMatrix(convertMatrixToSO3(rotationMatrix));

		common::verifyMatrix(Mat3f{ computedRotationMatrix }, rotationMatrix);
	}

	INSTANTIATE_TEST_CASE_P(Default, ConvertMatToSO3Test, testing::Values(
		Mat3f::identity(),
		Mat3f::rotation(10, 20, 30),
		Mat3f::rotation(-10, -20, -30),
		Mat3f::rotation(1, 2, 3)
	));


	//######################################################################################################
	// Pose inversion test

	struct AffinePoseInversionTest : testing::Test, testing::WithParamInterface<Mat4f> {};

	TEST_P(AffinePoseInversionTest, Default) {
		const Mat4f poseMatrix = GetParam();
		AffinePosef affinePose{ poseMatrix };

		auto affinePoseInverse = invertPose(affinePose, Unsigned2Type<PoseType::AFFINE>());
		Mat4f poseInverseMatrix{ Mat3f{ affinePoseInverse }, Vec3f{ affinePoseInverse[I<9>()], affinePoseInverse[I<10>()], affinePoseInverse[I<11>()] } };
	
		common::verifyMatrix(poseMatrix * poseInverseMatrix, Mat4f::identity());
		common::verifyMatrix(poseInverseMatrix * poseMatrix, Mat4f::identity());
	}

	INSTANTIATE_TEST_CASE_P(Default, AffinePoseInversionTest, testing::Values(
		Mat4f::identity(),
		Mat4f::rotation(10, 20, 30),
		Mat4f::rotation(-10, -20, -30),
		Mat4f::rotation(1, 2, 3),
		Mat4f::translation(10, 20, 30),
		Mat4f::translation(-10, -20, -30),
		Mat4f::translation(1, 2, 3),
		Mat4f::pose(10, 20, 30, 1, 2, 3),
		Mat4f::pose(10, 20, 30, -1, -2, -3),
		Mat4f::pose(10, -20, 30, -1, -2, -3)
	));

	struct SO3wTPoseInversionTest : testing::Test, testing::WithParamInterface<Mat4f> {};

	TEST_P(SO3wTPoseInversionTest, Default) {
		const Mat4f poseMatrix = GetParam();
		SO3wTf SO3wTPose{ poseMatrix };

		auto SO3wTPoseInverse = invertPose(SO3wTPose, Unsigned2Type<PoseType::SO3wT>());
		Mat4f poseInverseMatrix{ SO3wTf{ Vec6f{ 
			SO3wTPoseInverse[I<0>()], SO3wTPoseInverse[I<1>()], SO3wTPoseInverse[I<2>()], 
			SO3wTPoseInverse[I<3>()], SO3wTPoseInverse[I<4>()], SO3wTPoseInverse[I<5>()]
		} }.matrix() };

		common::verifyMatrix(poseMatrix * poseInverseMatrix, Mat4f::identity());
		common::verifyMatrix(poseInverseMatrix * poseMatrix, Mat4f::identity());
	}

	INSTANTIATE_TEST_CASE_P(Default, SO3wTPoseInversionTest, testing::Values(
		Mat4f::identity(),
		Mat4f::rotation(10, 20, 30),
		Mat4f::rotation(-10, -20, -30),
		Mat4f::rotation(1, 2, 3),
		Mat4f::translation(10, 20, 30),
		Mat4f::translation(-10, -20, -30),
		Mat4f::translation(1, 2, 3),
		Mat4f::pose(10, 20, 30, 1, 2, 3),
		Mat4f::pose(10, 20, 30, -1, -2, -3),
		Mat4f::pose(10, -20, 30, -1, -2, -3)
	));

	struct RigidPoseInversionTest : testing::Test, testing::WithParamInterface<Mat4f> {};

	TEST_P(RigidPoseInversionTest, Default) {
		const Mat4f poseMatrix = GetParam();
		RigidPosef quatPose{ poseMatrix };

		auto quatPoseInverse = invertPose(quatPose, Unsigned2Type<PoseType::QUATERNION>());
		Mat4f poseInverseMatrix{ RigidPosef{
			Quatf{	quatPoseInverse[I<0>()], quatPoseInverse[I<1>()], quatPoseInverse[I<2>()], quatPoseInverse[I<3>()] },
			Vec3f{ quatPoseInverse[I<4>()], quatPoseInverse[I<5>()], quatPoseInverse[I<6>()] }
		}.matrix() };

		common::verifyMatrix(poseMatrix * poseInverseMatrix, Mat4f::identity());
		common::verifyMatrix(poseInverseMatrix * poseMatrix, Mat4f::identity());
	}

	INSTANTIATE_TEST_CASE_P(Default, RigidPoseInversionTest, testing::Values(
		Mat4f::identity(),
		Mat4f::rotation(10, 20, 30),
		Mat4f::rotation(-10, -20, -30),
		Mat4f::rotation(1, 2, 3),
		Mat4f::translation(10, 20, 30),
		Mat4f::translation(-10, -20, -30),
		Mat4f::translation(1, 2, 3),
		Mat4f::pose(10, 20, 30, 1, 2, 3),
		Mat4f::pose(10, 20, 30, -1, -2, -3),
		Mat4f::pose(10, -20, 30, -1, -2, -3)
	));

} // namespace pose_operations_tests