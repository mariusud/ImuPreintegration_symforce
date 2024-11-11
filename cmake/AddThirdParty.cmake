# ------------------------------------------------------------------------------
# fmtlib
# ------------------------------------------------------------------------------

find_package(fmt 8...<9 REQUIRED)
if (NOT fmt_FOUND)
  message(STATUS "fmt not found, adding with FetchContent")
  function(add_fmt)
    set(FMT_INSTALL ON CACHE INTERNAL "fmt should create an install target")
    FetchContent_Declare(
      fmtlib
      URL https://github.com/fmtlib/fmt/archive/8.0.1.zip
      URL_HASH SHA256=6747442c189064b857336007dd7fa3aaf58512aa1a0b2ba76bf1182eefb01025
    )
    set(CMAKE_POSITION_INDEPENDENT_CODE True)
    FetchContent_MakeAvailable(fmtlib)
  endfunction()

  add_fmt()
else()
  message(STATUS "fmt found: ${fmt_VERSION}")
endif()


get_target_property(FMT_INTERFACE_INCLUDES fmt::fmt
                    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "FMT_INTERFACE_INCLUDES: ${FMT_INTERFACE_INCLUDES}")

# ------------------------------------------------------------------------------
# spdlog
# ------------------------------------------------------------------------------

find_package(spdlog 1.9.0 REQUIRED)
if (NOT spdlog_FOUND)
  message(STATUS "spdlog not found, adding with FetchContent")
  function(add_spdlog)
    set(SPDLOG_INSTALL ON CACHE INTERNAL "spdlog should create an install target")
    set(SPDLOG_FMT_EXTERNAL ON CACHE INTERNAL "spdlog shouldn't use its bundled fmtlib")
    set(CMAKE_POSITION_INDEPENDENT_CODE True)
    FetchContent_Declare(
      spdlog
      URL https://github.com/gabime/spdlog/archive/v1.9.2.zip
      URL_HASH SHA256=130bd593c33e2e2abba095b551db6a05f5e4a5a19c03ab31256c38fa218aa0a6
    )
    FetchContent_MakeAvailable(spdlog)
  endfunction()

  add_spdlog()
else()
  message(STATUS "spdlog found: ${spdlog_VERSION}")
endif()

get_target_property(SPDLOG_INTERFACE_INCLUDES spdlog::spdlog
                    INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "SPDLOG_INTERFACE_INCLUDES: ${SPDLOG_INTERFACE_INCLUDES}")


# ------------------------------------------------------------------------------
# PCL
# ------------------------------------------------------------------------------

find_package(VTK REQUIRED)
find_package(PCL)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

