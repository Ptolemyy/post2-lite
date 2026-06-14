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
    src/core/ksp_vehicle_site_import.cpp
    src/core/finite_difference.cpp
    src/core/nlp_evaluator.cpp
    src/core/nlp_problem.cpp
    src/core/optimization.cpp
    src/core/optimizer.cpp
    src/core/coordinates.cpp
    src/core/frames.cpp
    src/core/projection.cpp
    src/core/qp_solver.cpp
    src/core/remote_service.cpp
    src/core/simulation_driver.cpp
    src/core/state_log.cpp
    src/core/trajectory_service.cpp
    src/environment/atmosphere.cpp
    src/integrators/dopri5.cpp
    src/integrators/ode_integrator.cpp
    src/propagation/builtin_events.cpp
    src/propagation/engine_performance.cpp
    src/propagation/force_model.cpp
    src/propagation/force_model_set.cpp
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
        src/gui/chart_panel.cpp
        src/gui/opengl_scene_renderer.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/generated/gui_resources.rc
    )
    target_compile_definitions(post2_gui PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(post2_gui PRIVATE post2_core gdi32 user32 comdlg32 opengl32)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(post2_smoke_test
        tests/smoke_test.cpp
    )
    target_link_libraries(post2_smoke_test PRIVATE post2_core)
    add_test(NAME post2_smoke_test COMMAND post2_smoke_test)

    add_executable(post2_frames_test
        tests/frames_test.cpp
    )
    target_link_libraries(post2_frames_test PRIVATE post2_core)
    add_test(NAME post2_frames_test COMMAND post2_frames_test)

    add_executable(post2_engine_performance_test
        tests/engine_performance_test.cpp
    )
    target_link_libraries(post2_engine_performance_test PRIVATE post2_core)
    add_test(NAME post2_engine_performance_test COMMAND post2_engine_performance_test)

    add_executable(post2_integrator_test
        tests/integrator_test.cpp
    )
    target_link_libraries(post2_integrator_test PRIVATE post2_core)
    add_test(NAME post2_integrator_test COMMAND post2_integrator_test)

    add_executable(post2_optimization_components_test
        tests/optimization_components_test.cpp
    )
    target_link_libraries(post2_optimization_components_test PRIVATE post2_core)
    add_test(NAME post2_optimization_components_test COMMAND post2_optimization_components_test)

    add_executable(post2_phase_termination_and_events_test
        tests/phase_termination_and_events_test.cpp
    )
    target_link_libraries(post2_phase_termination_and_events_test PRIVATE post2_core)
    add_test(NAME post2_phase_termination_and_events_test
        COMMAND post2_phase_termination_and_events_test)

    add_executable(post2_ksp_vehicle_site_import_test
        tests/ksp_vehicle_site_import_test.cpp
    )
    target_link_libraries(post2_ksp_vehicle_site_import_test PRIVATE post2_core)
    add_test(
        NAME post2_ksp_vehicle_site_import_test
        COMMAND post2_ksp_vehicle_site_import_test
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/ksp_vehicle_site_v1_wrapped.json
    )

    add_test(
        NAME post2_shell_ksp_vehicle_site_import_smoke
        COMMAND post2_shell simulate
            --import-ksp-vehicle-site ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/ksp_vehicle_site_v1_wrapped.json
            --duration 1
            --dt 1
            --no-csv
            --no-svg
    )
endif()
