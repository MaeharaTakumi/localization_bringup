cmake_minimum_required(VERSION 3.8)
project(localization_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

# EKF を起動しないときに NDT・lidarslam の解から TF map -> base を出す
install(PROGRAMS scripts/pose_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
