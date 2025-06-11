if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

# Project settings
cmake_minimum_required(VERSION 3.15)

project(MeshReconstruction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS src/*.cpp)

set(BOOST_INCLUDEDIR "D:/dev/boost_1_86_0")
set(BOOST_LIBRARYDIR "D:/dev/boost_1_86_0/stage/lib")
set(CGAL_DIR "D:/dev/CGAL-5.6.2/CGAL-5.6.2")
set(GMP_DIR "D:/dev/CGAL-5.6.2/auxiliary/gmp")

find_package(OpenVDB REQUIRED)

add_executable(MeshReconstruction ${SRC})

target_compile_options(MeshReconstruction PRIVATE /bigobj)

target_include_directories(MeshReconstruction PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${BOOST_INCLUDEDIR} ${CGAL_DIR}/include ${GMP_DIR}/include)

target_link_directories(MeshReconstruction PRIVATE ${BOOST_LIBRARYDIR} ${GMP_DIR}/lib)

target_link_libraries(MeshReconstruction PRIVATE OpenVDB::openvdb libboost_filesystem-vc143-mt-gd-x64-1_86.lib libgmp-10.lib libmpfr-4.lib)