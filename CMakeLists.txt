# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(pc_annotation LANGUAGES C CXX)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

find_package(pybind11 REQUIRED)

include_directories(include)

pybind11_add_module(pc_annotation
  src/main.cpp
  src/DrawGeometry.cpp
  src/VisualizerForAnnotation.cpp
  src/VisualizerWithAnnotation.cpp
  src/shader/PointCloudRendererForAnnotation.cpp
  src/shader/PhongShaderForAnnotation.cpp
  src/shader/ShaderWrapperForAnnotation.cpp)

target_link_libraries(pc_annotation
  PRIVATE Open3D::Open3D
  PRIVATE pybind11::module)

# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
if(WIN32)
    get_target_property(open3d_type Open3D::Open3D TYPE)
    if(open3d_type STREQUAL "SHARED_LIBRARY")
        message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
        add_custom_command(TARGET Draw POST_BUILD
                           COMMAND ${CMAKE_COMMAND} -E copy
                                   ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                   ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
    endif()
endif()

# install
set(CMD_SITE_PKG "import site; import sys;sys.stdout.write(site.getusersitepackages())")

execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "${CMD_SITE_PKG}"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES_INSTALL_DIR
    )

set(PIP_INSTALL OFF)
if(PIP_INSTALL)
    install(TARGETS pc_annotation DESTINATION .)
else()
    # install to the site pakcage
    install(TARGETS pc_annotation DESTINATION ${PYTHON_SITE_PACKAGES_INSTALL_DIR})
    message("destination site-packages:" ${PYTHON_SITE_PACKAGES_INSTALL_DIR})
endif()