cmake_minimum_required(VERSION 2.8.3)
project(micromouse_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(
	DIRECTORY gazebo launch meshes urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)