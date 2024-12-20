cmake_minimum_required(VERSION 3.8)
project(safety_monitoring_system)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(safety_monitoring_system
  src/safety_monitoring_system.cpp
  src/clustering.cpp
)

ament_export_include_directories(include)

ament_auto_package(INSTALL_TO_SHARE)
