#pragma once
#include "matrix_lib/matrix_structures/MatrixStructuresInclude.h"
#include "matrix_lib/pose_structures/PoseStructuresInclude.h"

namespace matrix_lib {
	namespace common {

		/**
		 * Verifies the epsilon equality of two 2D/3D/4D vectors.
		 */
		template<typename T>
		inline void verifyVector(const Vec2<T>& estimate, const Vec2<T>& groundtruth, float eps = 0.000001f) {
			EXPECT_NEAR(estimate[0], groundtruth[0], eps);
			EXPECT_NEAR(estimate[1], groundtruth[1], eps);
		}

		template<typename T>
		inline void verifyVector(const Vec3<T>& estimate, const Vec3<T>& groundtruth, float eps = 0.000001f) {
			EXPECT_NEAR(estimate[0], groundtruth[0], eps);
			EXPECT_NEAR(estimate[1], groundtruth[1], eps);
			EXPECT_NEAR(estimate[2], groundtruth[2], eps);
		}

		template<typename T>
		inline void verifyVector(const Vec4<T>& estimate, const Vec4<T>& groundtruth, float eps = 0.000001f) {
			EXPECT_NEAR(estimate[0], groundtruth[0], eps);
			EXPECT_NEAR(estimate[1], groundtruth[1], eps);
			EXPECT_NEAR(estimate[2], groundtruth[2], eps);
			EXPECT_NEAR(estimate[3], groundtruth[3], eps);
		}

		template<typename T, int s>
		inline void verifyVector(const VecX<T, s>& estimate, const  VecX<T, s>& groundtruth, float eps = 0.000001f) {
			for (int i = 0; i < s; ++i) {
				EXPECT_NEAR(estimate[i], groundtruth[i], eps);
			}
		}


		/**
		 * Verifies the epsilon equality of two matrices.
		 */
		template<typename T>
		inline void verifyMatrix(const Mat2<T>& estimate, const Mat2<T>& groundtruth, float eps = 0.000001f) {
			for (int x = 0; x < 2; ++x) {
				for (int y = 0; y < 2; ++y) {
					EXPECT_NEAR(estimate(x, y), groundtruth(x, y), eps);
				}
			}
		}

		template<typename T>
		inline void verifyMatrix(const Mat3<T>& estimate, const Mat3<T>& groundtruth, float eps = 0.000001f) {
			for (int x = 0; x < 3; ++x) {
				for (int y = 0; y < 3; ++y) {
					EXPECT_NEAR(estimate(x, y), groundtruth(x, y), eps);
				}
			}
		}

		template<typename T>
		inline void verifyMatrix(const Mat4<T>& estimate, const Mat4<T>& groundtruth, float eps = 0.000001f) {
			for (int x = 0; x < 4; ++x) {
				for (int y = 0; y < 4; ++y) {
					EXPECT_NEAR(estimate(x, y), groundtruth(x, y), eps);
				}
			}
		}


		/**
		 * Verifies the epsilon equality of two poses. We can set different epsilons for translation
		 * difference (in meters) and rotation difference (in quaternions).
		 */
		inline void verifyPose(const RigidPosef& estimatedPose, const RigidPosef& groundtruthPose, float epsTranslation = 0.000001f, float epsRotation = 0.000001f) {
			common::verifyVector(estimatedPose.getTranslation(), groundtruthPose.getTranslation(), epsTranslation);
			common::verifyVector(estimatedPose.getQuaternion().imag(), groundtruthPose.getQuaternion().imag(), epsRotation);
			EXPECT_NEAR(estimatedPose.getQuaternion().real(), groundtruthPose.getQuaternion().real(), epsRotation);
		}

		inline void verifyRigidPose(const RigidPosef& estimatedPose, const RigidPosef& groundtruthPose, float epsTranslation = 0.000001f, float epsRotation = 0.000001f) {
			common::verifyVector(estimatedPose.getTranslation(), groundtruthPose.getTranslation(), epsTranslation);
			common::verifyVector(estimatedPose.getQuaternion().imag(), groundtruthPose.getQuaternion().imag(), epsRotation);
			EXPECT_NEAR(estimatedPose.getQuaternion().real(), groundtruthPose.getQuaternion().real(), epsRotation);
		}


		/**
		 * Verifies the epsilon equality of two poses. We can set different epsilons for translation
		 * difference (in meters) and rotation difference (per matrix element).
		 */
		inline void verifyPose(const AffinePosef& estimatedPose, const AffinePosef& groundtruthPose, float epsTranslation = 0.000001f, float epsRotation = 0.000001f) {
			common::verifyVector(groundtruthPose.getTranslation(), estimatedPose.getTranslation(), epsTranslation);
			common::verifyMatrix(groundtruthPose.getAffineMatrix(), estimatedPose.getAffineMatrix(), epsRotation);
		}

		inline void verifyAffinePose(const AffinePosef& estimatedPose, const AffinePosef& groundtruthPose, float epsTranslation = 0.000001f, float epsRotation = 0.000001f) {
			common::verifyVector(groundtruthPose.getTranslation(), estimatedPose.getTranslation(), epsTranslation);
			common::verifyMatrix(groundtruthPose.getAffineMatrix(), estimatedPose.getAffineMatrix(), epsRotation);
		}


		/**
		 * Base class for the data of parametrized tests (for printing out meaningful debug output).
		 */
		struct TestDescription {
			std::string description;

			TestDescription(const std::string& description_) : description{ description_ } { }
			virtual ~TestDescription() {}

			virtual std::ostream& print(std::ostream& os) const {
				return os << description;
			}
		};

	} // namespace common
} // namespace matrix_lib
