# FindSymforce.cmake
find_path(
  SYMFORCE_INCLUDE_DIR
  PATHS "/usr/local/include;/usr/include"
  NAMES symforce)

find_path(
  SYM_INCLUDE_DIR
  PATHS "/usr/local/include;/usr/include"
  NAMES sym)

find_library(
  SYMFORCE_OPT_PATH
  NAMES symforce_opt
  PATHS "/usr/local/lib;/usr/lib"
  PATH_SUFFIXES lib)

find_library(
  SYMFORCE_CHOLESKY_PATH
  NAMES symforce_cholesky
  PATHS "/usr/local/lib;/usr/lib"
  PATH_SUFFIXES lib)

find_library(
  SYMFORCE_SLAM_PATH
  NAMES symforce_slam
  PATHS "/usr/local/lib;/usr/lib"
  PATH_SUFFIXES lib)

find_library(
  SYMFORCE_GEN_PATH
  NAMES symforce_gen
  PATHS "/usr/local/lib;/usr/lib"
  PATH_SUFFIXES lib)

message(STATUS "SYMFORCE_INCLUDE_DIR: ${SYMFORCE_INCLUDE_DIR}")
message(STATUS "SYM_INCLUDE_DIR: ${SYM_INCLUDE_DIR}")
message(STATUS "SYMFORCE_OPT_PATH: ${SYMFORCE_OPT_PATH}")
message(STATUS "SYMFORCE_CHOLESKY_PATH: ${SYMFORCE_CHOLESKY_PATH}")
message(STATUS "SYMFORCE_SLAM_PATH: ${SYMFORCE_SLAM_PATH}")
message(STATUS "SYMFORCE_GEN_PATH: ${SYMFORCE_GEN_PATH}")

if(NOT SYMFORCE_INCLUDE_DIR
   OR NOT EXISTS ${SYMFORCE_INCLUDE_DIR}
   OR NOT SYMFORCE_OPT_PATH
   OR NOT EXISTS ${SYMFORCE_OPT_PATH}
   OR NOT SYMFORCE_CHOLESKY_PATH
   OR NOT EXISTS ${SYMFORCE_CHOLESKY_PATH}
   OR NOT SYMFORCE_SLAM_PATH
   OR NOT EXISTS ${SYMFORCE_SLAM_PATH}
   OR NOT SYMFORCE_GEN_PATH
   OR NOT EXISTS ${SYMFORCE_GEN_PATH}
   OR NOT SYM_INCLUDE_DIR
   OR NOT EXISTS ${SYM_INCLUDE_DIR})
  set(SYMFORCE_FOUND FALSE)
  message(WARNING "Failed to locate Symforce.")
else()
  set(SYMFORCE_FOUND TRUE)
endif()

if(${SYMFORCE_FOUND})
  if(NOT TARGET symforce::opt)
    add_library(symforce::opt SHARED IMPORTED)
    set_target_properties(symforce::opt PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                   ${SYMFORCE_INCLUDE_DIR})
    set_target_properties(symforce::opt PROPERTIES IMPORTED_LOCATION
                                                   ${SYMFORCE_OPT_PATH})
  endif()

  if(NOT TARGET symforce::cholesky)
    add_library(symforce::cholesky SHARED IMPORTED)
    set_target_properties(
      symforce::cholesky PROPERTIES IMPORTED_LOCATION ${SYMFORCE_CHOLESKY_PATH})
  endif()

  if(NOT TARGET symforce::slam)
  add_library(symforce::slam SHARED IMPORTED)
  set_target_properties(
    symforce::slam PROPERTIES IMPORTED_LOCATION ${SYMFORCE_SLAM_PATH})
endif()

  if(NOT TARGET symforce::gen)
    add_library(symforce::gen SHARED IMPORTED)
    set_target_properties(symforce::gen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                   ${SYM_INCLUDE_DIR})
    set_target_properties(symforce::gen PROPERTIES IMPORTED_LOCATION
                                                   ${SYMFORCE_GEN_PATH})
  endif()
endif()

mark_as_advanced(SYMFORCE_INCLUDE_DIR SYM_INCLUDE_DIR)
