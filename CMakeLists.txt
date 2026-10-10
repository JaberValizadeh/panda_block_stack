cmake_minimum_required(VERSION 3.0.2)
project(panda_block_stack)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS scripts/reset_blocks.py scripts/stack_blocks.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch worlds urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
