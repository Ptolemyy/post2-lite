cmake_minimum_required(VERSION 3.20)

project(post2_lite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(post2_core STATIC
    src/core/case_config_io.cpp
    src/core/control_models.cpp
    src/core/io.cpp
    src/core/json.cpp
    src/core/optimization.cpp
    src/core/coordinates.cpp
    src/core/projection.cpp
    src/core/remote_service.cpp
    src/core/simulation_driver.cpp
    src/core/state_log.cpp
    src/core/trajectory_service.cpp
    src/integrators/ode_integrator.cpp
    src/propagation/force_models.cpp
    src/propagation/vehicle_consumption.cpp
    src/vehicle/runtime_state.cpp
    src/vehicle/vehicle.cpp
    src/vehicle/vehicle_config_io.cpp
)

target_include_directories(post2_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(WIN32)
    target_compile_definitions(post2_core PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(post2_core PUBLIC ws2_32)
endif()

add_executable(post2_shell
    src/shell/main.cpp
)
target_link_libraries(post2_shell PRIVATE post2_core)

add_executable(post2_core_server
    src/server/main.cpp
)
target_link_libraries(post2_core_server PRIVATE post2_core)

if(WIN32)
    set(POST2_GUI_RESOURCE_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/src/gui/resource.h")
    set(POST2_EARTH_TEXTURE_BMP "${CMAKE_CURRENT_SOURCE_DIR}/assets/gui/earth_blue_marble_2048.bmp")
    configure_file(
        src/gui/resources.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/generated/gui_resources.rc
        @ONLY
    )

    add_executable(post2_gui WIN32
        src/gui/main_win32.cpp
        src/gui/software_renderer_3d.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/generated/gui_resources.rc
    )
    target_compile_definitions(post2_gui PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(post2_gui PRIVATE post2_core gdi32 user32 comdlg32)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(post2_smoke_test
        tests/smoke_test.cpp
    )
    target_link_libraries(post2_smoke_test PRIVATE post2_core)
    add_test(NAME post2_smoke_test COMMAND post2_smoke_test)
endif()
