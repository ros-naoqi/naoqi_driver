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
qi_create_gtest( dynamic_callback_test "dynamic_callback.cpp" )
qi_use_lib( dynamic_callback_test GTEST ROS QI )
qi_create_gtest( recorder_callback_test "recorder_callback_test.cpp" )
qi_use_lib( recorder_callback_test GTEST QI)
target_link_libraries(
  recorder_callback_test
  alrosbridge
)
qi_create_gtest( converter_memory_test "converter_memory_test.cpp" )
qi_use_lib( converter_memory_test GTEST QI)
target_link_libraries(
  converter_memory_test
  alrosbridge
)
qi_create_gtest( event_registration_test "event_registration_test.cpp" )
qi_use_lib( event_registration_test GTEST QI)
target_link_libraries(
  event_registration_test
  alrosbridge
)
qi_create_gtest( small_test "small_test.cpp" )
qi_use_lib( small_test GTEST QI )
target_link_libraries(
  small_test
  alrosbridge
)
#qi_create_bin(concept_test "concept_test.cpp")
#qi_use_lib(concept_test BOOST)
