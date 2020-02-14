#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	TEST(PoseArraysTest, ASO3wTf) {
		sASO3wTf sSO3wT;
		sSO3wT.allocate(3);

		iASO3wTf iSO3wT;
		iSO3wT.wrap(sSO3wT.getPointerList());

		for (int i = 0; i < 3; ++i) {
			iSO3wT.rotation()[i].x = i * 1.f;
			iSO3wT.rotation()[i].y = i * 2.f;
			iSO3wT.rotation()[i].z = i * 3.f;
			iSO3wT.rotation()[i].w = i * 4.f;

			iSO3wT.translation()[i].x = -i * 1.f;
			iSO3wT.translation()[i].y = -i * 2.f;
			iSO3wT.translation()[i].z = -i * 3.f;
			iSO3wT.translation()[i].w = -i * 4.f;
		}

		for (int i = 0; i < 3; ++i) {
			EXPECT_FLOAT_EQ(iSO3wT.rotation()[i].x, i * 1.f);
			EXPECT_FLOAT_EQ(iSO3wT.rotation()[i].y, i * 2.f);
			EXPECT_FLOAT_EQ(iSO3wT.rotation()[i].z, i * 3.f);
			EXPECT_FLOAT_EQ(iSO3wT.rotation()[i].w, i * 4.f);
			
			EXPECT_FLOAT_EQ(iSO3wT.translation()[i].x, -i * 1.f);
			EXPECT_FLOAT_EQ(iSO3wT.translation()[i].y, -i * 2.f);
			EXPECT_FLOAT_EQ(iSO3wT.translation()[i].z, -i * 3.f);
			EXPECT_FLOAT_EQ(iSO3wT.translation()[i].w, -i * 4.f);
		}
	}

	TEST(PoseArraysTest, AQuaternionPosef) {
		sAQuaternionPosef sQuaternionPose;
		sQuaternionPose.allocate(3);

		iAQuaternionPosef iQuaternionPose;
		iQuaternionPose.wrap(sQuaternionPose.getPointerList());

		for (int i = 0; i < 3; ++i) {
			iQuaternionPose.rotation()[i].x = i * 1.f;
			iQuaternionPose.rotation()[i].y = i * 2.f;
			iQuaternionPose.rotation()[i].z = i * 3.f;
			iQuaternionPose.rotation()[i].w = i * 4.f;

			iQuaternionPose.translation()[i].x = -i * 1.f;
			iQuaternionPose.translation()[i].y = -i * 2.f;
			iQuaternionPose.translation()[i].z = -i * 3.f;
			iQuaternionPose.translation()[i].w = -i * 4.f;
		}

		for (int i = 0; i < 3; ++i) {
			EXPECT_FLOAT_EQ(iQuaternionPose.rotation()[i].x, i * 1.f);
			EXPECT_FLOAT_EQ(iQuaternionPose.rotation()[i].y, i * 2.f);
			EXPECT_FLOAT_EQ(iQuaternionPose.rotation()[i].z, i * 3.f);
			EXPECT_FLOAT_EQ(iQuaternionPose.rotation()[i].w, i * 4.f);

			EXPECT_FLOAT_EQ(iQuaternionPose.translation()[i].x, -i * 1.f);
			EXPECT_FLOAT_EQ(iQuaternionPose.translation()[i].y, -i * 2.f);
			EXPECT_FLOAT_EQ(iQuaternionPose.translation()[i].z, -i * 3.f);
			EXPECT_FLOAT_EQ(iQuaternionPose.translation()[i].w, -i * 4.f);
		}
	}

	TEST(PoseArraysTest, AAffinePosef) {
		sAAffinePosef sAAffinePose;
		sAAffinePose.allocate(3);

		iAAffinePosef iAAffinePose;
		iAAffinePose.wrap(sAAffinePose.getPointerList());

		for (int i = 0; i < 3; ++i) {
			iAAffinePose.rotation().xrow()[i].x = i * 1.f;
			iAAffinePose.rotation().xrow()[i].y = i * 2.f;
			iAAffinePose.rotation().xrow()[i].z = i * 3.f;
			iAAffinePose.rotation().xrow()[i].w = i * 4.f;
			iAAffinePose.rotation().yrow()[i].x = i * 1.f * 2.f;
			iAAffinePose.rotation().yrow()[i].y = i * 2.f * 2.f;
			iAAffinePose.rotation().yrow()[i].z = i * 3.f * 2.f;
			iAAffinePose.rotation().yrow()[i].w = i * 4.f * 2.f;
			iAAffinePose.rotation().zrow()[i].x = i * 1.f * 3.f;
			iAAffinePose.rotation().zrow()[i].y = i * 2.f * 3.f;
			iAAffinePose.rotation().zrow()[i].z = i * 3.f * 3.f;
			iAAffinePose.rotation().zrow()[i].w = i * 4.f * 3.f;

			iAAffinePose.translation()[i].x = -i * 1.f;
			iAAffinePose.translation()[i].y = -i * 2.f;
			iAAffinePose.translation()[i].z = -i * 3.f;
			iAAffinePose.translation()[i].w = -i * 4.f;
		}

		for (int i = 0; i < 3; ++i) {
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().xrow()[i].x, i * 1.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().xrow()[i].y, i * 2.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().xrow()[i].z, i * 3.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().xrow()[i].w, i * 4.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().yrow()[i].x, i * 1.f * 2.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().yrow()[i].y, i * 2.f * 2.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().yrow()[i].z, i * 3.f * 2.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().yrow()[i].w, i * 4.f * 2.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().zrow()[i].x, i * 1.f * 3.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().zrow()[i].y, i * 2.f * 3.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().zrow()[i].z, i * 3.f * 3.f);
			EXPECT_FLOAT_EQ(iAAffinePose.rotation().zrow()[i].w, i * 4.f * 3.f);

			EXPECT_FLOAT_EQ(iAAffinePose.translation()[i].x, -i * 1.f);
			EXPECT_FLOAT_EQ(iAAffinePose.translation()[i].y, -i * 2.f);
			EXPECT_FLOAT_EQ(iAAffinePose.translation()[i].z, -i * 3.f);
			EXPECT_FLOAT_EQ(iAAffinePose.translation()[i].w, -i * 4.f);
		}
	}

} // namespace pose_structures_tests