cmake_minimum_required(VERSION 3.10)

# Set the project name
project(grad_test)
# Find the Eigen3 package
find_package(Eigen3 REQUIRED)

# Include Eigen3 directories
include_directories(${EIGEN3_INCLUDE_DIR})
# Add the executable
add_executable(grad_test latGrad_test.cpp)

# Specify the C++ standard
set_target_properties(grad_test PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Include directories if needed
# target_include_directories(grad_test PRIVATE /path/to/include)

# Link libraries if needed
# target_link_libraries(grad_test PRIVATE some_library)