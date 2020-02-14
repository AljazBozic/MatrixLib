#pragma once
#include <vector>
#include <gtest/gtest.h>

#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;
using std::vector;

namespace pose_structures_tests {

	//######################################################################################################
	// Data structs for parameterized tests.

	struct PoseCompositionData : public common::TestDescription {
		Mat4f p1;
		Mat4f p2;

		PoseCompositionData(const std::string& description_, const Mat4f& p1_, const Mat4f& p2_) :
			common::TestDescription{ description_ }, p1{ p1_ }, p2{ p2_ }
		{ }

		friend std::ostream& operator<<(std::ostream& os, const PoseCompositionData& obj) { return obj.print(os); }
	};

	struct PoseLeftIncrementData : public common::TestDescription{
		Mat4f increment;
		Mat4f pose;

		PoseLeftIncrementData(const std::string& description_, const Mat4f& increment_, const Mat4f& pose_) :
			common::TestDescription{ description_ }, increment{ increment_ }, pose{ pose_ }
		{ }

		friend std::ostream& operator<<(std::ostream& os, const PoseLeftIncrementData& obj) { return obj.print(os); }
	};

	struct PoseApplicationData : public common::TestDescription {
		Mat4f pose;
		Vec3f point;

		PoseApplicationData(const std::string& description_, const Mat4f& pose_, const Vec3f& point_) :
			common::TestDescription{ description_ }, pose{ pose_ }, point{ point_ }
		{ }

		friend std::ostream& operator<<(std::ostream& os, const PoseApplicationData& obj) { return obj.print(os); }
	};

	struct PoseInterpolationData : public common::TestDescription {
		vector<Mat4f> poses;
		vector<float> weights;
		Mat4f		  interpolatedPose;

		PoseInterpolationData(const std::string& description_, const vector<Mat4f>& poses_, const vector<float>& weights_, const Mat4f& interpolatedPose_) :
			common::TestDescription{ description_ }, poses{ poses_ }, weights{ weights_ }, interpolatedPose{ interpolatedPose_ }
		{ }

		friend std::ostream& operator<<(std::ostream& os, const PoseInterpolationData& obj) { return obj.print(os); }
	};

	//######################################################################################################
	// Data for parametrized tests.

	#define POSE_COMPOSITION_VALUES testing::Values( \
		PoseCompositionData{ "id x id", Mat4f::identity(), Mat4f::identity() }, \
		PoseCompositionData{ "id x rot", Mat4f::identity(), Mat4f::rotation(12, 13, 14) }, \
		PoseCompositionData{ "id x trans", Mat4f::identity(), Mat4f::translation(1, 2, 3) }, \
		PoseCompositionData{ "id x pose", Mat4f::identity(), Mat4f::pose(12, 13, 14, 1, 2, 3) }, \
		PoseCompositionData{ "rot x pose", Mat4f::rotation(50, 90, 0), Mat4f::pose(12, 13, 14, 1, 2, 3) }, \
		PoseCompositionData{ "translation x pose", Mat4f::translation(50, 90, 0), Mat4f::pose(12, 13, 14, 1, 2, 3) }, \
		PoseCompositionData{ "pose x pose", Mat4f::pose(1, 2, 3, 4, 5, 6), Mat4f::pose(12, 13, 14, 1, 2, 3) } \
	)

	#define POSE_LEFT_INCREMENT_VALUES testing::Values( \
		PoseLeftIncrementData{ "id x id", Mat4f::identity(), Mat4f::identity() }, \
		PoseLeftIncrementData{ "id x rot", Mat4f::identity(), Mat4f::rotation(12, 13, 14) }, \
		PoseLeftIncrementData{ "id x trans", Mat4f::identity(), Mat4f::translation(1, 2, 3) }, \
		PoseLeftIncrementData{ "id x pose", Mat4f::identity(), Mat4f::pose(12, 13, 14, 1, 2, 3) }, \
		PoseLeftIncrementData{ "rot x pose", Mat4f::rotation(50, 90, 0), Mat4f::pose(12, 13, 14, 1, 2, 3) }, \
		PoseLeftIncrementData{ "translation x pose", Mat4f::translation(50, 90, 0), Mat4f::pose(12, 13, 14, 1, 2, 3) }, \
		PoseLeftIncrementData{ "pose x pose", Mat4f::pose(1, 2, 3, 4, 5, 6), Mat4f::pose(12, 13, 14, 1, 2, 3) } \
	)

	#define POSE_APPLICATION_VALUES testing::Values( \
		PoseApplicationData{ "id x v1", Mat4f::identity(), Vec3f{ 1, 0, 0 } }, \
		PoseApplicationData{ "rot x v1", Mat4f::rotation(12, 13, 14), Vec3f{ 1, 0, 0 } }, \
		PoseApplicationData{ "trans x v1", Mat4f::translation(1, 2, 3), Vec3f{ 1, 0, 0 } }, \
		PoseApplicationData{ "pose x v1", Mat4f::pose(12, 13, 14, 1, 2, 3), Vec3f{ 1, 0, 0 } }, \
		PoseApplicationData{ "id x v2", Mat4f::identity(), Vec3f{ 1, 0, 0 } }, \
		PoseApplicationData{ "rot x v2", Mat4f::rotation(12, 13, 14), Vec3f{ 10.f, -3.f, 13.f } }, \
		PoseApplicationData{ "trans x v2", Mat4f::translation(1, 2, 3), Vec3f{ 10.f, -3.f, 13.f } }, \
		PoseApplicationData{ "pose x v2", Mat4f::pose(12, 13, 14, 1, 2, 3), Vec3f{ 10.f, -3.f, 13.f } } \
	)

} // namespace pose_structures_tests