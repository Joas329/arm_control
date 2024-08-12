cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# ********************* Cpp Stuff ****************

include_directories(include)
include_directories(description)
include_directories(arm_control)

# Create cpp executables
# add_executable(cpp_exe src/cpp_node.cpp)
# ament_target_dependencies(cpp_exe rclcpp)


# Install cpp executables
# install(TARGETS
#   cpp_exe
#   DESTINATION lib/${PROJECT_NAME}
# )

# **************** Python Stuff *****************

ament_python_install_package(${PROJECT_NAME})

# Install Python executables

#install(PROGRAMS
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY description/mesh
  DESTINATION share/${PROJECT_NAME}/description
  FILES_MATCHING PATTERN "*.STL"
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
