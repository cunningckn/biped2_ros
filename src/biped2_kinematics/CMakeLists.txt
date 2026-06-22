cmake_minimum_required(VERSION 3.8)
project(biped2_kinematics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/foot_fk_publisher
  scripts/foot_lift_demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
