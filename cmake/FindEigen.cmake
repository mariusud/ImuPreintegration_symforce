# FindEigen.cmake
find_path(
  EIGEN_INCLUDE_DIR
  NAMES signature_of_eigen3_matrix_library
  PATHS "/usr/local/include;/usr/include"
  PATH_SUFFIXES eigen3)

if(NOT EIGEN_INCLUDE_DIR OR NOT EXISTS ${EIGEN_INCLUDE_DIR})
  set(EIGEN_FOUND FALSE)
  message(WARNING "Failed to locate Eigen.")
else()
  set(EIGEN_FOUND TRUE)
  message(STATUS "Eigen found: ${EIGEN_INCLUDE_DIR}")
endif()

set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})

# Create a target for eigen w/ the correct interface include directories. See
# https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/
if(${EIGEN_FOUND})
  if(NOT TARGET Eigen::Eigen)
    add_library(Eigen::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                  "${EIGEN_INCLUDE_DIRS}")
  endif()
endif()
