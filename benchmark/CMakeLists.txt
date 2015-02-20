# to use those utilities, you need to compile the message type
find_package( qibuild )

qi_create_bin(
  simple_publisher
  src/simple_publisher.cpp
)

qi_create_bin(
  simple_subscriber
  src/simple_subscriber.cpp
)

qi_use_lib( simple_publisher ROS )
qi_use_lib( simple_subscriber ROS )
