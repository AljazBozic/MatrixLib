project(matrix_lib_unit_tests CXX)

# Add preprocessor definitions.
# add_definitions(-DCOMPILE_CUDA)

# Add dependencies.
find_package(GTest CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

# Include all files
file(GLOB root "*.*")            
file(GLOB matrix_structures "matrix_structures/*.*")       
file(GLOB pose_structures "pose_structures/*.*")    
file(GLOB utils "utils/*.*")    

source_group("root" FILES ${root})
source_group("matrix_structures" FILES ${matrix_structures})
source_group("pose_structures" FILES ${pose_structures})
source_group("utils" FILES ${utils})

# Add executable.
add_executable(matrix_lib_unit_tests ${root} ${timing} ${matrix_structures} ${pose_structures} ${utils})
target_link_libraries(matrix_lib_unit_tests GTest::gtest GTest::gtest_main GTest::gmock GTest::gmock_main ${Boost_SERIALIZATION_LIBRARY})
set_target_properties(matrix_lib_unit_tests PROPERTIES LINKER_LANGUAGE CXX)