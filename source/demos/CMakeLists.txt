project(control_libraries_demos)

get_target_property(STATE_REPRESENTATION_INCLUDE_DIR state_representation INCLUDE_DIRECTORIES)
get_target_property(DYNAMICAL_SYSTEMS_INCLUDE_DIR dynamical_systems INCLUDE_DIRECTORIES)
get_target_property(ROBOT_MODEL_INCLUDE_DIR robot_model INCLUDE_DIRECTORIES)
get_target_property(CONTROLLERS_INCLUDE_DIR controllers INCLUDE_DIRECTORIES)

include_directories(
  include
  ${STATE_REPRESENTATION_INCLUDE_DIR}
  ${DYNAMICAL_SYSTEMS_INCLUDE_DIR}
  ${ROBOT_MODEL_INCLUDE_DIR}
  ${CONTROLLERS_INCLUDE_DIR}
)

set(DEMOS_SCRIPTS
    task_space_control_loop
)

foreach(SCRIPT ${DEMOS_SCRIPTS})
    add_executable(${SCRIPT} scripts/${SCRIPT}.cpp)
    target_link_libraries(${SCRIPT}
      state_representation
      dynamical_systems
      robot_model
      controllers
    )
    install(TARGETS ${SCRIPT}
      RUNTIME DESTINATION bin
    )
endforeach()