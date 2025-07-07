cmake_minimum_required(VERSION 3.8)
project(media_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
# install python package
ament_python_install_package(${PROJECT_NAME})

set(NODES
  media_manager/manager.py
)

set(FILES
  media_manager/bfid.py
  )

install(PROGRAMS
    ${NODES}
DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    ${FILES}
DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
ament_add_pytest_test(
  test_media_manager_pytest
  tests/test_service.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)
  #ament_lint_auto_find_test_dependencies()
endif()

ament_package()
