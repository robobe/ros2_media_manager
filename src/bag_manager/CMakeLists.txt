cmake_minimum_required(VERSION 3.8)
project(bag_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(ament_cmake_pytest REQUIRED)
# install python package
ament_python_install_package(${PROJECT_NAME})

set(NODES
  bag_manager/image2png.py
  bag_manager/my_bag.py
  )

set(FILES
  bag_manager/qos.py
  bag_manager/rosbag2.py
  )


install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    ${FILES}
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  
endif()

ament_package()
