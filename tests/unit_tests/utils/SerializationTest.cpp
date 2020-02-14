#include <ostream>
#include <iostream>
#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <gtest/gtest.h>

#include "matrix_lib/utils/Serialization.h"
#include "matrix_lib/utils/Constants.h"
#include "tests/unit_tests/utils/Common.h"

using namespace matrix_lib;

namespace utils_tests {

	//######################################################################################################
	// Serialization test.

	TEST(SerializationTest, Serialize_Vec2) {
		Vec2f v1 = Vec2f{ 1.f, 2.f };
		{
			std::ofstream ofs(OUTPUT("vec2.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << v1;
		}

		Vec2f v2;
		{
			std::ifstream ifs(OUTPUT("vec2.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> v2;
		}

		common::verifyVector(v1, v2);
	}

	TEST(SerializationTest, Serialize_Vec3) {
		Vec3f v1 = Vec3f{ 1.f, 2.f, 3.f };
		{
			std::ofstream ofs(OUTPUT("vec3.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << v1;
		}

		Vec3f v2;
		{
			std::ifstream ifs(OUTPUT("vec3.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> v2;
		}

		common::verifyVector(v1, v2);
	}

	TEST(SerializationTest, Serialize_Vec4) {
		Vec4f v1 = Vec4f{ 1.f, 2.f, 3.f, 4.f };
		{
			std::ofstream ofs(OUTPUT("vec4.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << v1;
		}

		Vec4f v2;
		{
			std::ifstream ifs(OUTPUT("vec4.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> v2;
		}

		common::verifyVector(v1, v2);
	}

	TEST(SerializationTest, Serialize_VecX) {
		Vec9f v1 = Vec9f{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f };
		{
			std::ofstream ofs(OUTPUT("vecX.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << v1;
		}

		Vec9f v2;
		{
			std::ifstream ifs(OUTPUT("vecX.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> v2;
		}

		common::verifyVector(v1, v2);
	}

	TEST(SerializationTest, Serialize_Mat2) {
		Mat2f m1 = Mat2f{ 1.f, 2.f, 3.f, 4.f };
		{
			std::ofstream ofs(OUTPUT("mat2.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << m1;
		}

		Mat2f m2;
		{
			std::ifstream ifs(OUTPUT("mat2.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> m2;
		}

		common::verifyMatrix(m1, m2);
	}

	TEST(SerializationTest, Serialize_Mat3) {
		Mat3f m1 = Mat3f{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f };
		{
			std::ofstream ofs(OUTPUT("mat3.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << m1;
		}

		Mat3f m2;
		{
			std::ifstream ifs(OUTPUT("mat3.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> m2;
		}

		common::verifyMatrix(m1, m2);
	}

	TEST(SerializationTest, Serialize_Mat4) {
		Mat4f m1 = Mat4f{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f };
		{
			std::ofstream ofs(OUTPUT("mat4.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << m1;
		}

		Mat4f m2;
		{
			std::ifstream ifs(OUTPUT("mat4.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> m2;
		}

		common::verifyMatrix(m1, m2);
	}

	TEST(SerializationTest, Serialize_SO3) {
		SO3f p1 = SO3f{ 1.f, 2.f, 3.f };
		{
			std::ofstream ofs(OUTPUT("so3.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << p1;
		}

		SO3f p2;
		{
			std::ifstream ifs(OUTPUT("so3.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> p2;
		}

		common::verifyVector(p1.getOmega(), p2.getOmega());
	}

	TEST(SerializationTest, Serialize_UnitQuaternion) {
		Quatf p1 = Quatf{ 1.f, 2.f, 3.f, 4.f };
		{
			std::ofstream ofs(OUTPUT("quat.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << p1;
		}

		Quatf p2;
		{
			std::ifstream ifs(OUTPUT("quat.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> p2;
		}

		EXPECT_FLOAT_EQ(p1.real(), p2.real());
		common::verifyVector(p1.imag(), p2.imag());
	}

	TEST(SerializationTest, Serialize_RigidPose) {
		RigidPosef p1 = RigidPosef{ Quatf{ 1.f, 2.f, 3.f, 4.f }, Vec3f{ 5.f, 6.f, 7.f } };
		{
			std::ofstream ofs(OUTPUT("rigid_pose.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << p1;
		}

		RigidPosef p2;
		{
			std::ifstream ifs(OUTPUT("rigid_pose.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> p2;
		}

		common::verifyPose(p1, p2);
	}

	TEST(SerializationTest, Serialize_SO3wT) {
		SO3wTf p1 = SO3wTf{ Vec6f{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f } };
		{
			std::ofstream ofs(OUTPUT("so3wt.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << p1;
		}

		SO3wTf p2;
		{
			std::ifstream ifs(OUTPUT("so3wt.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> p2;
		}

		common::verifyVector(p1.getAxisAngle().getOmega(), p2.getAxisAngle().getOmega());
		common::verifyVector(p1.getTranslation(), p2.getTranslation());
	}

	TEST(SerializationTest, Serialize_AffinePose) {
		AffinePosef p1 = AffinePosef{ Mat3f{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f }, Vec3f{ 10.f, 11.f, 12.f } };
		{
			std::ofstream ofs(OUTPUT("affine_pose.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << p1;
		}

		AffinePosef p2;
		{
			std::ifstream ifs(OUTPUT("affine_pose.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> p2;
		}

		common::verifyPose(p1, p2);
	}

	TEST(SerializationTest, Serialize_AffineIncrement) {
		AffineIncrementf p1 = AffineIncrementf{ Mat3f{ 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f }, Vec3f{ 10.f, 11.f, 12.f } };
		{
			std::ofstream ofs(OUTPUT("affine_inc.bin"));
			boost::archive::binary_oarchive oa(ofs);
			oa << p1;
		}

		AffineIncrementf p2;
		{
			std::ifstream ifs(OUTPUT("affine_inc.bin"));
			boost::archive::binary_iarchive ia(ifs);
			ia >> p2;
		}

		common::verifyMatrix(p1.getAffineMatrix(), p2.getAffineMatrix());
		common::verifyVector(p1.getTranslation(), p2.getTranslation());
	}

} // namespace utils_tests



