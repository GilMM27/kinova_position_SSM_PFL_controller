cmake_minimum_required(VERSION 3.20)
project(KinovaPositionSSMPFLController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CXX_DISABLE_WERROR 1)

find_package(mc_rtc REQUIRED)
find_package(human_robot_distance_api 2 CONFIG REQUIRED)
option(WITH_SMPLH_CONTACT_LOGGING "Log SMPL-H contact and torque estimates" OFF)
if(WITH_SMPLH_CONTACT_LOGGING)
  find_path(SMPLH_CONTACT_INCLUDE_DIR mc_mujoco/SmplhContactForces.h REQUIRED)
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/KinovaPositionSSMPFLController.in.yaml
  "${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaPositionSSMPFLController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/KinovaPositionSSMPFLController.yaml"
  DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

option(BUILD_TESTING "Build tests that require the Kinova robot module" OFF)
include(CTest)
if(BUILD_TESTING)
  add_executable(kinova_position_ssm_pfl_smoke_test
    tests/kinova_position_ssm_pfl_smoke.cpp)
  target_link_libraries(kinova_position_ssm_pfl_smoke_test PRIVATE
    KinovaPositionSSMPFLController mc_rtc::mc_control)
  add_test(NAME kinova_position_ssm_pfl_smoke
    COMMAND kinova_position_ssm_pfl_smoke_test
      "${CMAKE_CURRENT_SOURCE_DIR}/examples/ssm_pfl_example.yaml")
endif()
