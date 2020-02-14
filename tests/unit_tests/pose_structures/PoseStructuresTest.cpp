#include <gtest/gtest.h>

#include "matrix_lib/MatrixLibInclude.h"

using namespace matrix_lib;

namespace pose_structures_tests {

	TEST(PoseStructuresTest, ComposingTypes) {
		VecX<SO3f, 5> vSO3f;
		VecX<Quatf, 5> vQuatf;
		VecX<SO3wTf, 5> vSO3wTf;
		VecX<RigidPosef, 5> vRigidPosef;
		VecX<AffineIncrementf, 5> vAffineIncrementf;
		VecX<AffinePosef, 5> vAffinePosef;
	}

} // namespace pose_structures_tests