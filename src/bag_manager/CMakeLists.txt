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
)


install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  
endif()

ament_package()
