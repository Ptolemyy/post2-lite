cmake_minimum_required(VERSION 3.20)

project(post2_lite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(post2_core STATIC
    src/aero/aero_table.cpp
    src/aero/aero_model.cpp
    src/core/aero_table_build.cpp
    src/core/case_config_io.cpp
    src/core/control_models.cpp
    src/core/io.cpp
    src/core/json.cpp
    src/core/ksp_vehicle_site_import.cpp
    src/core/finite_difference.cpp
    src/core/gfold_solver.cpp
    src/core/nlp_evaluator.cpp
    src/core/nlp_problem.cpp
    src/core/optimization.cpp
    src/core/optimizer.cpp
    src/core/coordinates.cpp
    src/core/frames.cpp
    src/core/projection.cpp
    src/core/qp_solver.cpp
    src/core/reentry_solver.cpp
    src/core/remote_service.cpp
    src/core/simulation_driver.cpp
    src/core/state_log.cpp
    src/core/trajectory_service.cpp
    src/core/upfg.cpp
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

# Optional Clarabel conic solver for the re-entry burn guidance (milestone of
# the entry-burn feature). OFF by default so the standard build/tests need no
# clarabel_c.dll. Turn on with -DPOST2_USE_CLARABEL=ON; the headers + prebuilt
# import lib come from the vendored Clarabel.cpp under gfold2d/ (build it first
# per gfold2d/README: cargo build --release). When enabled, post2_core links
# the import lib PUBLIC and defines POST2_HAVE_CLARABEL, and every executable
# gets the DLL copied next to it (see the deploy loop at the end of this file).
option(POST2_USE_CLARABEL "Build the Clarabel-based re-entry burn guidance" OFF)
set(POST2_CLARABEL_DLL "")
if(POST2_USE_CLARABEL)
    set(POST2_CLARABEL_ROOT
        "${CMAKE_CURRENT_SOURCE_DIR}/gfold2d/external/Clarabel.cpp"
        CACHE PATH "Root of the vendored Clarabel.cpp")
    set(POST2_CLARABEL_INC "${POST2_CLARABEL_ROOT}/include")
    set(POST2_CLARABEL_LIBDIR "${POST2_CLARABEL_ROOT}/rust_wrapper/target/release")
    if(EXISTS "${POST2_CLARABEL_INC}/clarabel.h" AND
       EXISTS "${POST2_CLARABEL_LIBDIR}/clarabel_c.dll.lib")
        target_include_directories(post2_core PUBLIC "${POST2_CLARABEL_INC}")
        target_link_libraries(post2_core PUBLIC "${POST2_CLARABEL_LIBDIR}/clarabel_c.dll.lib")
        target_compile_definitions(post2_core PUBLIC POST2_HAVE_CLARABEL)
        set(POST2_CLARABEL_DLL "${POST2_CLARABEL_LIBDIR}/clarabel_c.dll")
        message(STATUS "Clarabel: enabled (${POST2_CLARABEL_LIBDIR})")
    else()
        message(WARNING
            "POST2_USE_CLARABEL=ON but Clarabel not found under ${POST2_CLARABEL_ROOT} "
            "(build the vendored Clarabel.cpp first). Building without it.")
    endif()
endif()

# Copies clarabel_c.dll next to a target's executable when Clarabel is enabled.
# No-op otherwise. post2_core links the import lib PUBLIC, so every executable
# that uses post2_core needs the DLL on its PATH at run time.
function(post2_deploy_clarabel target)
    if(POST2_CLARABEL_DLL)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${POST2_CLARABEL_DLL}" "$<TARGET_FILE_DIR:${target}>")
    endif()
endfunction()

add_executable(post2_shell
    src/shell/main.cpp
)
target_link_libraries(post2_shell PRIVATE post2_core)

add_executable(post2_core_server
    src/server/main.cpp
)
target_link_libraries(post2_core_server PRIVATE post2_core)

# Headless KSP re-import + aero-table regeneration for case files.
add_executable(post2_regen_aero
    src/tools/regen_aero.cpp
)
target_link_libraries(post2_regen_aero PRIVATE post2_core)
if(WIN32)
    target_compile_definitions(post2_regen_aero PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

# Standalone kRPC ascent-guidance player. Requires the vcpkg krpc package
# (configure with the vcpkg toolchain, e.g. `cmake --preset vcpkg`). When krpc
# is not available the player is skipped so the core build still works.
find_package(krpc CONFIG QUIET)
if(krpc_FOUND)
    add_executable(post2_player
        src/player/main.cpp
        src/player/guidance_script.cpp
    )
    target_link_libraries(post2_player PRIVATE post2_core krpc::krpc)
    if(WIN32)
        target_compile_definitions(post2_player PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    endif()
    message(STATUS "post2_player: enabled (krpc found)")

    # Standalone grid-fin aerodynamics test tool (its own subproject).
    add_subdirectory(gridfin_test)
    message(STATUS "gridfin_test: enabled (krpc found)")
else()
    message(STATUS "post2_player: skipped (krpc not found; use the vcpkg toolchain)")
endif()

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

    add_executable(post2_dynamics_dof_test
        tests/dynamics_dof_test.cpp
    )
    target_link_libraries(post2_dynamics_dof_test PRIVATE post2_core)
    add_test(NAME post2_dynamics_dof_test COMMAND post2_dynamics_dof_test)

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

    add_executable(post2_upfg_guidance_test
        tests/upfg_guidance_test.cpp
    )
    target_link_libraries(post2_upfg_guidance_test PRIVATE post2_core)
    add_test(NAME post2_upfg_guidance_test COMMAND post2_upfg_guidance_test)

    add_executable(post2_aero_model_test
        tests/aero_model_test.cpp
    )
    target_link_libraries(post2_aero_model_test PRIVATE post2_core)
    add_test(NAME post2_aero_model_test COMMAND post2_aero_model_test)

    add_executable(post2_reentry_solver_test
        tests/reentry_solver_test.cpp
    )
    target_link_libraries(post2_reentry_solver_test PRIVATE post2_core)
    add_test(NAME post2_reentry_solver_test COMMAND post2_reentry_solver_test)

    add_executable(post2_engine_selection_test
        tests/engine_selection_test.cpp
    )
    target_link_libraries(post2_engine_selection_test PRIVATE post2_core)
    add_test(NAME post2_engine_selection_test COMMAND post2_engine_selection_test)

    add_executable(post2_gfold_solver_test
        tests/gfold_solver_test.cpp
    )
    target_link_libraries(post2_gfold_solver_test PRIVATE post2_core)
    add_test(NAME post2_gfold_solver_test COMMAND post2_gfold_solver_test)

    add_executable(post2_aero_stage_table_test
        tests/aero_stage_table_test.cpp
    )
    target_link_libraries(post2_aero_stage_table_test PRIVATE post2_core)
    add_test(NAME post2_aero_stage_table_test COMMAND post2_aero_stage_table_test)

    # End-to-end (krpc-independent): core writes the guidance script, the player's
    # parser reads it back. Compiles the player parser directly so it needs no krpc.
    add_executable(post2_guidance_script_test
        tests/guidance_script_test.cpp
        src/player/guidance_script.cpp
    )
    target_link_libraries(post2_guidance_script_test PRIVATE post2_core)
    add_test(NAME post2_guidance_script_test COMMAND post2_guidance_script_test)

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

# Deploy clarabel_c.dll next to every executable when Clarabel is enabled. Must
# run after all targets are defined; guarded so it is a no-op in the default
# (Clarabel-off) build.
if(POST2_CLARABEL_DLL)
    foreach(_post2_exe
        post2_shell post2_core_server post2_regen_aero post2_player post2_gui
        post2_smoke_test post2_frames_test post2_engine_performance_test
        post2_integrator_test post2_dynamics_dof_test post2_optimization_components_test
        post2_phase_termination_and_events_test post2_upfg_guidance_test
        post2_aero_model_test post2_reentry_solver_test post2_engine_selection_test
        post2_gfold_solver_test
        post2_aero_stage_table_test post2_guidance_script_test
        post2_ksp_vehicle_site_import_test)
        if(TARGET ${_post2_exe})
            post2_deploy_clarabel(${_post2_exe})
        endif()
    endforeach()
endif()
