cmake_minimum_required(VERSION 3.10)
project(gazebo_grasp_plugin)

# Pure CMake package (NOT ament_cmake) to avoid the pyenv/python3 bootstrap clash.
# We manually register it in the ament index so get_package_share_directory()
# can locate the install prefix from the launch file.

find_package(gazebo REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED gazebo_grasp_fix.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

# Install the .so where the launch file's GAZEBO_PLUGIN_PATH points.
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib)

# ---- Manual ament index registration (normally done by ament_package()) ----
# Without this, get_package_prefix('gazebo_grasp_plugin') fails and the launch
# file cannot put this plugin's lib dir on GAZEBO_PLUGIN_PATH.
#
# The index marker must be an (empty) file NAMED AFTER THE PACKAGE at
#   share/ament_index/resource_index/packages/<package_name>
# The previous version installed the file as "package.xml" instead, so the marker
# was never found and the package stayed invisible to ROS despite building fine.
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker" "")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ament_index_marker"
        DESTINATION share/ament_index/resource_index/packages
        RENAME ${PROJECT_NAME})

# ament also expects share/<package_name>/package.xml to exist.
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
