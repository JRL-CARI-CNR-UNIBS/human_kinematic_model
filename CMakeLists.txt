#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)
get_project_name("package.xml" extracted_name extracted_version)
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
set(CMAKE_BUILD_TYPE Release)
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(ENABLE_TESTING           "ENABLE TESTING" ON)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(USE_ROS1                 "USE ROS1 FOR TESTING" OFF)
option(BUILD_EXAMPLE            "COMPILE THE EXAMPLE" OFF)

#############################
## END - OPTIONS           ##
#############################


# Python modules of the pip package (human_kinematic_model, human_kinematic_model_jax): copied into site-packages by
# a regular install; an editable install (python -m pip install -e .) installs a .pth file that puts scripts/ on
# sys.path instead, so that changes to the scripts are live without reinstalling
macro(hkm_install_python_modules)
  if(SKBUILD_STATE STREQUAL "editable")
    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/human_model_scripts.pth" "${CMAKE_CURRENT_SOURCE_DIR}/scripts\n")
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/human_model_scripts.pth" DESTINATION .)
  else()
    install(FILES scripts/human_kinematic_model.py scripts/human_kinematic_model_jax.py DESTINATION .)
  endif()
endmacro()

#############################
## PYTHON PACKAGE WITHOUT THE BINDING ##
#############################
# `python -m pip install .` (scikit-build-core, SKBUILD defined) on a system without the Python development headers
# (python3-dev / python3.X-dev): the binding cannot be compiled, but the JAX model and the python translation do not
# need it, so the package is installed without it (a warning; reinstall after installing the headers to add it).
# scikit-build-core passes the interpreter's include folder as Python_INCLUDE_DIR, which FindPython would use without
# checking that it exists (an obscure "patchlevel.h cannot be read" error).
if(SKBUILD AND NOT EXISTS "${Python_INCLUDE_DIR}/Python.h")
  message(WARNING "Python development headers not found (${Python_INCLUDE_DIR}/Python.h): installing human_model "
                  "WITHOUT the C++ binding human_model_binding (the JAX model and the python translation are "
                  "installed). To add the binding: sudo apt install python3-dev (or python3.X-dev for this Python "
                  "version), then reinstall.")
  hkm_install_python_modules()
  return()
endif()

#############################
## DEPENDENCIES            ##
#############################
# SKBUILD is defined when pip builds the Python package (`python -m pip install .`, scikit-build-core, see
# pyproject.toml): then only the Python package is built (binding with the library linked in, python modules), Eigen
# is downloaded if the system has none, and nothing is installed outside the wheel.
if(SKBUILD)
  find_package(Eigen3 QUIET COMPONENTS Geometry)
  if(NOT TARGET Eigen3::Eigen)
    message(STATUS "Eigen3 not found: downloading Eigen 3.4.0 (headers only)")
    include(FetchContent)
    # SOURCE_SUBDIR without a CMakeLists.txt: download only, Eigen's own CMake project is not configured
    FetchContent_Declare(eigen
      URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
      URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
      SOURCE_SUBDIR no_cmake)
    FetchContent_MakeAvailable(eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
  endif()
else()
  find_package(Eigen3 REQUIRED COMPONENTS Geometry)
endif()

cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE_TESTING USE_ROS1)

if(SKBUILD)
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
else()
  find_package(Python REQUIRED COMPONENTS
               Interpreter  # for the installation of the python package
               Development # for the c++ -> python bindings
  )
endif()
find_package(pybind11 REQUIRED CONFIG)

# detect virtualenv and set Pip args accordingly
if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
  set(_pip_args)
else()
  set(_pip_args "--user")
endif()
#############################
## END DEPENDENCIES        ##
#############################

#################
## Build       ##
#################
# Define the library: shared for C++ users; static inside the Python package, so that the binding is self-contained
if(SKBUILD)
  add_library(human_model STATIC src/human_model/human_model.cpp)
  set_target_properties(human_model PROPERTIES POSITION_INDEPENDENT_CODE ON)
else()
  add_library(human_model SHARED
    src/human_model/human_model.cpp)
endif()

# Set the include directories
target_include_directories(human_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #if BUILD_INTERFACE, add ${CMAKE_SOURCE_DIR}/include (headers taken by source directory)
  $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
)

# Link other libraries
target_link_libraries(human_model
  PUBLIC
  Eigen3::Eigen
)

# Create an alias
add_library(human_model::human_model ALIAS human_model)

# Add tests
if(ENABLE_TESTING)
  add_executable(human_model_test test/test_fk_ik.cpp)
  target_link_libraries(human_model_test
    PUBLIC
    human_model::human_model
    ${GTEST_LIBRARIES}
  )
  target_compile_definitions(human_model_test
    PUBLIC
    PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  cnr_target_compile_options(human_model_test)

  # Speed test of the fk -> ik -> fk loop (no gtest, no prints)
  add_executable(human_model_benchmark test/benchmark_fk_ik.cpp)
  target_link_libraries(human_model_benchmark
    PUBLIC
    human_model::human_model
  )
  cnr_target_compile_options(human_model_benchmark)
endif()

# Python bindings
pybind11_add_module(human_model_binding src/human_model/bindings.cpp)

# Set the include directories
target_include_directories(human_model_binding PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #if BUILD_INTERFACE, add ${CMAKE_SOURCE_DIR}/include (headers taken by source directory)
  $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
)

# Link other libraries
target_link_libraries(human_model_binding
  PRIVATE
  human_model::human_model
)

# if(BUILD_EXAMPLE)
# ## example of usage
#   add_executable(test test/test.cpp)
#   target_link_libraries(test
#     PUBLIC
#     human_model::human_model)
#   target_compile_definitions(rdyn_speed_test PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
#   cnr_target_compile_options(rdyn_speed_test)
# endif()
#################
## END - Build ##
#################

### #############
### ## Install ##
### #############
if(SKBUILD)
  # Python package: the binding and the python modules at the top level of site-packages (import human_model_binding,
  # human_kinematic_model, human_kinematic_model_jax); CMAKE_INSTALL_PREFIX is the wheel's platlib folder
  install(TARGETS human_model_binding LIBRARY DESTINATION .)
  hkm_install_python_modules()
  return()
endif()

# Install the include directory
install(
  DIRECTORY include/human_model
  DESTINATION include
)

# Install the test
if(ENABLE_TESTING)
  install(
    TARGETS human_model_test human_model_benchmark
    EXPORT human_modelTargets
    RUNTIME DESTINATION bin/human_model
  )
endif()

# Install the library and the bindings
install(
  TARGETS
    human_model
    human_model_binding
  EXPORT human_modelTargets
  ARCHIVE DESTINATION lib/human_model
  LIBRARY DESTINATION lib/human_model
)

# Install the export so that other CMake projects can find the library
install(EXPORT human_modelTargets
  DESTINATION "share/human_model/cmake"
  NAMESPACE human_model::
  FILE human_modelTargets.cmake
)

# Create a package configuration file and version file + install them
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/human_modelConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/human_modelConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/human_modelConfig.cmake
  INSTALL_DESTINATION "share/human_model/cmake"
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/human_modelConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/human_modelConfigVersion.cmake
  DESTINATION "share/human_model/cmake"
)

# Install the Python package in development mode (live changes); it builds the binding through scikit-build-core, with
# SKBUILD defined, so this line is not reached again
execute_process(COMMAND ${Python_EXECUTABLE} -m pip install ${_pip_args} -e ${CMAKE_CURRENT_LIST_DIR})

# make uninstall
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)