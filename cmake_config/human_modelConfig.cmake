include(CMakeFindDependencyMacro)

find_dependency(rdyn_core REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/human_modelTargets.cmake")
