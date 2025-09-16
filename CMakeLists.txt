# 使用 vcpkg toolchain
if(DEFINED ENV{VCPKG_ROOT})
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

cmake_minimum_required(VERSION 3.15)
project(MeshReconstruction LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE SRC CONFIGURE_DEPENDS src/*.cpp)

# 找库
find_package(OpenVDB REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS filesystem system)

# 添加可执行文件
add_executable(MeshReconstruction ${SRC})

target_compile_options(MeshReconstruction PRIVATE /bigobj)
target_include_directories(MeshReconstruction PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接库
target_link_libraries(MeshReconstruction PRIVATE 
    OpenVDB::openvdb
    CGAL::CGAL
    CGAL::CGAL_Core
    Boost::filesystem
    Boost::system
)