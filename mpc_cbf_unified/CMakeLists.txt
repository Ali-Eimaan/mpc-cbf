cmake_minimum_required(VERSION 3.16)
project(mpc_cbf_unified)

# SKELETON NOTE ---------------------------------------------------------------
# The build graph below is complete and intended to be kept as-is. Only the
# blocks marked TODO(deepseek) need editing once acados-generated sources exist.
# See IMPLEMENTATION_GUIDE.md §2 for the acados discovery contract.
# -----------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# --- acados discovery --------------------------------------------------------
# acados is not an ament package; it is located through the ACADOS_SOURCE_DIR
# environment variable (set by acados' own env.sh). When it is absent the
# library still builds, but the solver back-ends compile to a stub that returns
# SolverStatus::kNotInitialized. This keeps `colcon build` green on machines
# without acados and keeps CI honest about what is actually exercised.
set(ACADOS_ROOT $ENV{ACADOS_SOURCE_DIR})
if(ACADOS_ROOT)
  message(STATUS "mpc_cbf_unified: acados found at ${ACADOS_ROOT}")
  set(MPC_CBF_WITH_ACADOS TRUE)
  # TODO(deepseek): after running codegen, point these at the generated tree.
  #   set(ACADOS_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../codegen/c_generated_code)
  #   file(GLOB ACADOS_GENERATED_SOURCES ${ACADOS_GENERATED_DIR}/*.c
  #                                      ${ACADOS_GENERATED_DIR}/**/*.c)
else()
  message(WARNING
    "mpc_cbf_unified: ACADOS_SOURCE_DIR is not set -- building solvers as stubs. "
    "Source <acados>/env.sh before building to enable the real QP back-end.")
  set(MPC_CBF_WITH_ACADOS FALSE)
endif()

# --- core library ------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/mpc_cbf_solver.cpp
  src/tube_mpc_cbf_solver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

if(MPC_CBF_WITH_ACADOS)
  target_compile_definitions(${PROJECT_NAME} PRIVATE MPC_CBF_WITH_ACADOS=1)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${ACADOS_ROOT}/include
    ${ACADOS_ROOT}/include/blasfeo/include
    ${ACADOS_ROOT}/include/hpipm/include
  )
  target_link_directories(${PROJECT_NAME} PRIVATE ${ACADOS_ROOT}/lib)
  target_link_libraries(${PROJECT_NAME} PRIVATE acados hpipm blasfeo)
endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp)

# --- node executable ---------------------------------------------------------
add_executable(mpc_cbf_node src/mpc_cbf_node.cpp)
target_link_libraries(mpc_cbf_node PRIVATE ${PROJECT_NAME})
ament_target_dependencies(mpc_cbf_node PUBLIC
  rclcpp rclcpp_components std_msgs geometry_msgs nav_msgs sensor_msgs
  visualization_msgs diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs
)

# --- install -----------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS mpc_cbf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

# --- tests -------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Copyright/licence linters are noise for a single-author research repo.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_mpc_cbf_feasibility test/test_mpc_cbf_feasibility.cpp)
  if(TARGET test_mpc_cbf_feasibility)
    target_link_libraries(test_mpc_cbf_feasibility ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_tube_mpc_robustness test/test_tube_mpc_robustness.cpp)
  if(TARGET test_tube_mpc_robustness)
    target_link_libraries(test_tube_mpc_robustness ${PROJECT_NAME})
  endif()

  ament_add_pytest_test(test_recursive_feasibility test/test_recursive_feasibility.py
    TIMEOUT 600
  )
endif()

ament_package()
