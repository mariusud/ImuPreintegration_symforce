cmake_minimum_required(VERSION 3.19...3.25)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.27)
  message(FATAL_ERROR
    "SymForce does not support CMake >=3.27: https://github.com/symforce-org/symforce/issues/356"
    )
endif()

project(IMU-preintegration)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Set the installation directory path
set(CMAKE_INSTALL_PREFIX "/home/marius/Development/symforce")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include(FetchContent)
include(cmake/AddThirdParty.cmake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen REQUIRED)
find_package(Symforce REQUIRED)

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})


add_executable(run_imu_preintegration src/main.cc src/data.cc)

# Add dependencies
add_dependencies(
    run_imu_preintegration
    Eigen::Eigen
    fmt::fmt
    spdlog::spdlog
    symforce::opt
    symforce::gen
    symforce::cholesky
    symforce::slam
)

# Link libraries
target_link_libraries(
    run_imu_preintegration
    ${PCL_LIBRARIES}
    Eigen::Eigen
    fmt::fmt
    spdlog::spdlog
    symforce::opt
    symforce::gen
    symforce::cholesky
    symforce::slam
)

# Install target
install(TARGETS run_imu_preintegration DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)