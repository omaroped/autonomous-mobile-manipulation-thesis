cmake_minimum_required(VERSION 3.22)
project(limo_cobot_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# MoveItConfigsBuilder reads this hidden file from the installed share dir to
# locate the URDF (in limo_car) and SRDF. It is NOT inside config/, so install
# it explicitly — otherwise the builder can't infer the robot description.
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
