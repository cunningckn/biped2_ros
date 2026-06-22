cmake_minimum_required(VERSION 3.8)
project(biped2_description)

find_package(ament_cmake REQUIRED)

find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(XACRO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/urdf/biped2.urdf.xacro)
set(URDF_FILE ${CMAKE_CURRENT_BINARY_DIR}/biped2.urdf)
set(MESH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/meshes)
set(RESOLVE_MESH_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/resolve_mesh_paths.py)

add_custom_command(
  OUTPUT ${URDF_FILE}
  COMMAND ${XACRO_EXECUTABLE} ${XACRO_FILE} -o ${URDF_FILE}
  COMMAND python3 ${RESOLVE_MESH_SCRIPT} ${URDF_FILE} ${MESH_DIR}
  DEPENDS ${XACRO_FILE} ${RESOLVE_MESH_SCRIPT}
  COMMENT "Generating biped2.urdf"
)

add_custom_target(generate_biped2_urdf ALL DEPENDS ${URDF_FILE})

install(FILES ${URDF_FILE}
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
