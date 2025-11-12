if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

cmake_minimum_required(VERSION 3.15)
project(MeshReconstruction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS src/*.cpp)


find_package(OpenVDB REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS filesystem system)


add_library(MeshReconstruction SHARED ${SRC})
add_executable(MeshReconstructionApp ${SRC})

target_compile_definitions(MeshReconstruction PRIVATE 
    MESHRECONSTRUCTION_EXPORTS
)

target_compile_options(MeshReconstruction PRIVATE /bigobj)
target_include_directories(MeshReconstruction PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(MeshReconstruction PRIVATE 
    OpenVDB::openvdb
    CGAL::CGAL
    CGAL::CGAL_Core
    Boost::filesystem
    Boost::system
)

target_compile_options(MeshReconstructionApp PRIVATE /bigobj)
target_include_directories(MeshReconstructionApp PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(MeshReconstructionApp PRIVATE 
    OpenVDB::openvdb
    CGAL::CGAL
    CGAL::CGAL_Core
    Boost::filesystem
    Boost::system
)