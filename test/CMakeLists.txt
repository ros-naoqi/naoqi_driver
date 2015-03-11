find_package( qibuild )

enable_testing()
qi_create_gtest( dcm_test "dcm_test.cpp" )
qi_use_lib( dcm_test GTEST ROS QI )
qi_create_gtest( robot_state_publisher_test "robot_state_publisher.cpp" )
qi_use_lib( robot_state_publisher_test GTEST ROS QI )
qi_create_gtest( recorder_test "recorder_test.cpp" )
qi_use_lib( recorder_test GTEST QI)
target_link_libraries(
  recorder_test
  alrosbridge
)
