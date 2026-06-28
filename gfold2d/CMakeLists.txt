cmake_minimum_required(VERSION 3.20)
project(gfold2d CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen comes from vcpkg (configure with its toolchain file).
find_package(Eigen3 CONFIG REQUIRED)

# Pre-built Clarabel C/C++ wrapper (see README: cargo build --release).
set(CLA       ${CMAKE_CURRENT_SOURCE_DIR}/external/Clarabel.cpp)
set(CLA_LIBDIR ${CLA}/rust_wrapper/target/release)

add_executable(gfold2d src/gfold2d.cpp)
target_compile_definitions(gfold2d PRIVATE _USE_MATH_DEFINES)   # M_PI on MSVC
target_include_directories(gfold2d PRIVATE ${CLA}/include)
target_link_libraries(gfold2d PRIVATE
    Eigen3::Eigen
    ${CLA_LIBDIR}/clarabel_c.dll.lib)        # import lib for clarabel_c.dll

# Put the Clarabel DLL next to the executable so it runs in place.
add_custom_command(TARGET gfold2d POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CLA_LIBDIR}/clarabel_c.dll $<TARGET_FILE_DIR:gfold2d>)
