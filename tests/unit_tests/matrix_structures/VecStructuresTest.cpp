#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"

using namespace matrix_lib;

namespace matrix_structures_tests {

	TEST(VecStructuresTest, ComposingTypes) {
		Vec2<Vec2f> v22;
		Vec3<Vec3f> v33;
		Vec4<Vec4f> v44;
		VecX<VecX<float, 5>, 5> v55;

		Vec2<Vec3f> v23;
		Vec3<Vec4f> v34;
		Vec4<VecX<float, 5>> v45;
		VecX<VecX<float, 5>, 6> v56;
	}

} // namespace matrix_structures_tests