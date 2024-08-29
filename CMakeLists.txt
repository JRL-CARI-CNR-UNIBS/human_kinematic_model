#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)
get_project_name("package.xml" extracted_name extracted_version)
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" OFF)
option(BUILD_EXAMPLE            "COMPILE THE EXAMPLE" OFF)
#############################
## END - OPTIONS           ##
#############################


#############################
## DEPENDANCIES            ##
#############################
find_package(rdyn_core REQUIRED)
#cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE)
#############################
## END DEPENDANCIES        ##
#############################

#################
## Build       ##
#################

add_library(human_model  SHARED
  src/human_model/human_model.cpp)
target_include_directories(human_model
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> #if BUILD_INTERFACE, add ${CMAKE_SOURCE_DIR}/include (headers taken by source directory)
    $<INSTALL_INTERFACE:include> # if INSTALL_INTERFACE, add header from install path
  )
#target_compile_definitions(human_model
#  PUBLIC
#  )

target_link_libraries(human_model
  PUBLIC
  rdyn_core::rdyn_core
)

add_library(human_model::human_model ALIAS human_model)


#add_executable(test test/test_fk_ik.cpp)
#target_link_libraries(test
#  PUBLIC
#  human_model::human_model)
#target_compile_definitions(test PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
#cnr_target_compile_options(test)



#if(BUILD_EXAMPLE)
### example of usage
#  add_executable(test test/test.cpp)
#  target_link_libraries(test
#    PUBLIC
#    human_model::human_model)
#  target_compile_definitions(rdyn_speed_test PUBLIC PROJECT_SRC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}")
#  cnr_target_compile_options(rdyn_speed_test)
#endif()
#################
## END - Build ##
#################

### #############
### ## Install ##
### #############


install(DIRECTORY include/human_model
  DESTINATION include)

install(
  TARGETS human_model
  EXPORT human_modelTargets
  ARCHIVE DESTINATION lib/human_model
  LIBRARY DESTINATION lib/human_model
  RUNTIME DESTINATION bin/human_model
)

install(EXPORT human_modelTargets
  DESTINATION "share/human_model/cmake"
  NAMESPACE human_model::
  FILE human_modelTargets.cmake
)


include(CMakePackageConfigHelpers)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/human_modelConfigVersion.cmake
  VERSION 0.0.1
  COMPATIBILITY AnyNewerVersion)

install(FILES
  ${CMAKE_SOURCE_DIR}/cmake_config/human_modelConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/human_modelConfigVersion.cmake
  DESTINATION "share/human_model/cmake")


# make uninstall
add_custom_target("uninstall" COMMENT "Uninstall installed files")
add_custom_command(
    TARGET "uninstall"
    POST_BUILD
    COMMENT "Uninstall files with install_manifest.txt"
    COMMAND xargs rm -vf < install_manifest.txt || echo Nothing in
            install_manifest.txt to be uninstalled!
)
