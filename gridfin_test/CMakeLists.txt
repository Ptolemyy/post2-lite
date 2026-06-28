# gridfin_test -- standalone kRPC aero-glide landing-guidance test tool.
#
# Two nested loops steer a descending, UNPOWERED Falcon 9 first stage to a
# target lat/lon/alt using only the four grid fins:
#   * outer: sampling-based (CEM) nonlinear MPC over the point-mass aero-glide
#     dynamics, using the parent project's real CD/CL aero table;
#   * inner: own attitude PID pointing the nose via Control.set_pitch/yaw/roll.
# Connects to the kRPC server on rpc port 30000 / stream port 30001.
#
# Self-contained at runtime: links only krpc::krpc (+ threads). The parent aero
# table reader (src/aero/aero_table.cpp, standard-library only) is compiled in
# directly so there is no post2_core / Clarabel DLL dependency. Added to the
# build by the root CMakeLists inside the find_package(krpc) block via
# add_subdirectory(gridfin_test).

add_executable(gridfin_test
    main.cpp
    aero_glide_mpc.cpp
    fin_mpc.cpp
    ${CMAKE_SOURCE_DIR}/src/aero/aero_table.cpp
)

target_compile_features(gridfin_test PRIVATE cxx_std_17)
target_include_directories(gridfin_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(gridfin_test PRIVATE krpc::krpc)

find_package(Threads QUIET)
if(Threads_FOUND)
    target_link_libraries(gridfin_test PRIVATE Threads::Threads)
endif()

if(WIN32)
    target_compile_definitions(gridfin_test PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
endif()

# Ship the booster-only (grid-fins-deployed) aero table next to the executable
# so the default --table resolves regardless of the working directory.
add_custom_command(TARGET gridfin_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/aero_table_booster.csv
            $<TARGET_FILE_DIR:gridfin_test>/aero_table_booster.csv
    COMMENT "Copying booster aero table next to gridfin_test.exe")
