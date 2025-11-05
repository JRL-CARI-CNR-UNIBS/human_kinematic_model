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


#############################
## DEPENDENCIES            ##
#############################
find_package(Eigen3 REQUIRED COMPONENTS Geometry)

cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE_TESTING USE_ROS1)

find_package(Python REQUIRED COMPONENTS 
             Interpreter  # for the installation of the python package
             Development # for the c++ -> python bindings
)
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
# Define the library
add_library(human_model SHARED
  src/human_model/human_model.cpp)

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
# Install the include directory
install(
  DIRECTORY include/human_model
  DESTINATION include
)

# Install the test
if(ENABLE_TESTING)
  install(
    TARGETS human_model_test
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

# Install the Python package in development mode (live changes)
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