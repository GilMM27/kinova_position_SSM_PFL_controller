set(controller_SRC KinovaPositionSSMPFLController.cpp)
set(controller_HDR KinovaPositionSSMPFLController.h SSMPFLConstraint.h SSMPFLMath.h api.h)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
if(WITH_SMPLH_CONTACT_LOGGING)
  target_compile_definitions(${PROJECT_NAME} PUBLIC WITH_SMPLH_CONTACT_LOGGING)
  target_include_directories(${PROJECT_NAME} PRIVATE ${SMPLH_CONTACT_INCLUDE_DIR})
endif()
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm human_robot_distance_api::human_robot_distance_api)
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS "-DKinovaPositionSSMPFLController_EXPORTS")
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_controller lib.cpp)
target_link_libraries(${PROJECT_NAME}_controller PRIVATE ${PROJECT_NAME})
