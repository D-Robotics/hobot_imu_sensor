# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(imu_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## pkg-config libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

if(PLATFORM_X3)
  message("build platform X3")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)

  include_directories(include
    include/imu_sensor/ 
  )

  set(BASE_LIBRARIES pthread yaml-cpp)
  file(GLOB_RECURSE BMI088_FILES
          ${PROJECT_SOURCE_DIR}/src/x3/sensors/*.cc
          ${PROJECT_SOURCE_DIR}/src/x3/sensors/*.cpp
  )
  add_library(bmi088 SHARED ${BMI088_FILES})
  target_link_libraries(bmi088 ${BASE_LIBRARIES})

  file(GLOB_RECURSE CPP_FILES
          ${PROJECT_SOURCE_DIR}/src/x3/*.cc
          ${PROJECT_SOURCE_DIR}/src/x3/*.cpp
  )
  # Install executables
  install(
    TARGETS ${PROJECT_NAME} bmi088
    RUNTIME DESTINATION lib/${PROJECT_NAME}/
    LIBRARY DESTINATION lib/${PROJECT_NAME}/
  )
elseif(PLATFORM_X5)
  message("build platform X5")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
  include_directories(include
    include/imu_sensor/ 
    include
  )

  set(BASE_LIBRARIES pthread yaml-cpp)

  file(GLOB_RECURSE CPP_FILES
    ${PROJECT_SOURCE_DIR}/src/x5/sensors/*.cc
    ${PROJECT_SOURCE_DIR}/src/x5/sensors/*.cpp
    ${PROJECT_SOURCE_DIR}/src/x5/*.cc
    ${PROJECT_SOURCE_DIR}/src/x5/*.cpp
  )
else()
  message("invalid platform, build platform X3 default")
  add_definitions(-DPLATFORM_X3)
  set(PREFIX_PATH x3)
endif()  


add_executable(${PROJECT_NAME}
  ${CPP_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${BASE_LIBRARIES}
  yaml-cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  sensor_msgs
  yaml_cpp_vendor
  )

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/${PREFIX_PATH}/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  launch/${PREFIX_PATH}/
  DESTINATION share/${PROJECT_NAME}/launch/
)

ament_package()
