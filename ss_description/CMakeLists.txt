cmake_minimum_required(VERSION 3.5)
project(ss_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(xacro)
ament_export_dependencies(urdf)
ament_package()