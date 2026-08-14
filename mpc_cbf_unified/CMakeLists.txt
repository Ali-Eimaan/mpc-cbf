cmake_minimum_required(VERSION 3.22...3.31)
project(mpc_cbf_unified)

# SKELETON NOTE ---------------------------------------------------------------
# The build graph is complete. acados-generated sources (from
# codegen/generate_mpc_cbf_solver.py --all, run at the repo root) are globbed
# under ../c_generated_code when ACADOS_SOURCE_DIR is set; without them the
# library builds but the solver back-ends compile to stubs.
# See .deepseek/03_BUILD_SYSTEM.md §3.2 for the acados discovery contract.
# -----------------------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # -Wshadow/-Wconversion are deliberate: see .deepseek/03_BUILD_SYSTEM.md §3.5.
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
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
  # Generated solvers live in <repo-root>/c_generated_code/<solver_name>/ —
  # the DEFAULT_OUTPUT_DIR of codegen/generate_*_solver.py when run from the
  # repo root (see .deepseek/05_CODEGEN.md §5.6). GLOB_RECURSE pulls in the
  # flat acados export tree (model/, cost/, constraints/, acados_solver_*.c).
  # CONFIGURE_DEPENDS re-runs the glob on every build so a codegen step that
  # runs just before colcon build is picked up without a reconfigure.
  set(ACADOS_GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../c_generated_code)
  file(GLOB_RECURSE ACADOS_GENERATED_SOURCES CONFIGURE_DEPENDS
    ${ACADOS_GENERATED_DIR}/*.c)
  # Each acados export ships a standalone main_<solver>.c demo (its own main()).
  # It must not be linked into the library — it would collide with the other
  # solvers' mains and the node's own entry point.
  list(FILTER ACADOS_GENERATED_SOURCES EXCLUDE REGEX "/main_[^/]*\\.c$")
  if(NOT ACADOS_GENERATED_SOURCES)
    message(WARNING
      "mpc_cbf_unified: ACADOS_SOURCE_DIR is set but no generated sources found "
      "under ${ACADOS_GENERATED_DIR} — run "
      "'python3 codegen/generate_mpc_cbf_solver.py --all' first; solvers will "
      "fail to link until then.")
  endif()
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
  # acados + generated-code include dirs are SYSTEM (03_BUILD_SYSTEM.md §3.3):
  # their headers are not -Wconversion-clean and must not silence our own
  # warnings. Each generated solver exports into its own subdirectory, so the
  # per-solver dirs join the include path individually (the sources include
  # "acados_solver_<name>.h" by bare filename).
  target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    ${ACADOS_ROOT}/include
    ${ACADOS_ROOT}/include/blasfeo/include
    ${ACADOS_ROOT}/include/hpipm/include
  )
  if(ACADOS_GENERATED_SOURCES)
    # GLOB_RECURSE lists full paths; the unique parent dirs are the solver
    # export roots, and each one joins the (SYSTEM) include path.
    set(_gen_dirs "")
    foreach(_gen_src IN LISTS ACADOS_GENERATED_SOURCES)
      get_filename_component(_gen_dir "${_gen_src}" DIRECTORY)
      list(APPEND _gen_dirs "${_gen_dir}")
    endforeach()
    list(REMOVE_DUPLICATES _gen_dirs)
    target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${_gen_dirs})
  endif()
  target_link_directories(${PROJECT_NAME} PRIVATE ${ACADOS_ROOT}/lib)
  target_link_libraries(${PROJECT_NAME} PRIVATE acados hpipm blasfeo)
  if(ACADOS_GENERATED_SOURCES)
    # The generated C sources are compiled into this library: loadAcadosApi
    # binds their entry points directly (no dlopen), so every solver the C++
    # side can request must be present in the link.
    target_sources(${PROJECT_NAME} PRIVATE ${ACADOS_GENERATED_SOURCES})
  endif()
endif()

# ament_target_dependencies is not available on lyrical (§3.4): use namespaced
# targets. rclcpp is linked only for its logging macros (the library stays
# ROS-message-free).
target_link_libraries(${PROJECT_NAME} PUBLIC rclcpp::rclcpp)

# --- node executable ---------------------------------------------------------
add_executable(mpc_cbf_node src/mpc_cbf_node.cpp)
target_link_libraries(mpc_cbf_node PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
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

# §3.7: the rclpy simulator/analysis nodes must be installed for `ros2 run` and
# the launch files to find them. Forgetting this block produces a launch
# failure that names the executable but not the cause.
install(PROGRAMS
  scripts/sim_double_integrator.py
  scripts/sim_bicycle.py
  scripts/sim_quadrotor_planar.py
  scripts/track_reference.py
  scripts/log_min_barrier.py
  DESTINATION lib/${PROJECT_NAME})

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

  # Test-only parity harness for A7 (§11.5). The pytest finds it through
  # MPC_CBF_CPP_SOLVE_CLI; ament_add_pytest_test accepts generator expressions
  # in the ENV list (deferred until test time).
  add_executable(cpp_solve_cli test/cpp_solve_cli.cpp)
  target_link_libraries(cpp_solve_cli PRIVATE ${PROJECT_NAME})

  ament_add_pytest_test(test_recursive_feasibility test/test_recursive_feasibility.py
    TIMEOUT 600
    ENV "MPC_CBF_CPP_SOLVE_CLI=$<TARGET_FILE:cpp_solve_cli>"
  )
endif()

ament_package()
