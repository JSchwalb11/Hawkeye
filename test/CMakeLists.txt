# Synthetic ray injectors and their scored checkers.
#
# Nothing here is ever linked into the viewer. The injectors emit over the wire
# like any vehicle, so the tests exercise the real ingest path instead of
# reaching around it into the map API.

set(FIXTURE_WORKDIR ${CMAKE_CURRENT_BINARY_DIR}/fixture-runs)

add_library(hawkeye-fixture-support STATIC
    geom.c
    truth.c
    orientation_basis.c
    canvas.c
    ortho_render.c
)
target_include_directories(hawkeye-fixture-support PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_include_directories(hawkeye-fixture-support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
if(UNIX AND NOT APPLE)
    target_link_libraries(hawkeye-fixture-support PUBLIC m)
endif()

add_executable(ray_injector ray_injector.c)
target_link_libraries(ray_injector PRIVATE hawkeye-fixture-support hawkeye-core)
target_include_directories(ray_injector PRIVATE
    ${CMAKE_SOURCE_DIR}/src ${MAVLINK_DIR} ${MAVLINK_DIR}/common)
target_compile_definitions(ray_injector PRIVATE MAVLINK_COMM_NUM_BUFFERS=256)
if(WIN32)
    target_link_libraries(ray_injector PRIVATE ws2_32)
endif()

add_executable(map_checker map_checker.c)
target_link_libraries(map_checker PRIVATE hawkeye-fixture-support hawkeye-core)
target_include_directories(map_checker PRIVATE
    ${CMAKE_SOURCE_DIR}/src ${MAVLINK_DIR} ${MAVLINK_DIR}/common)
target_compile_definitions(map_checker PRIVATE MAVLINK_COMM_NUM_BUFFERS=256)
if(WIN32)
    target_link_libraries(map_checker PRIVATE ws2_32)
endif()

# Each fixture isolates one failure mode. Scale trims the two heavy ones on
# machines where CI time matters; the durations they assert on are unchanged.
set(HAWKEYE_FIXTURES
    empty
    ground
    wall
    corridor
    orientations
    moving
    two-origins
    disagreement
    clocks
    vanishing
    cone
    weak
    firehose
    endurance
)

foreach(fixture ${HAWKEYE_FIXTURES})
    set(fixture_scale 1)
    if(fixture STREQUAL "firehose")
        set(fixture_scale ${HAWKEYE_FIXTURE_FIREHOSE_SCALE})
    endif()
    add_test(NAME fixture_${fixture}
        COMMAND ${CMAKE_COMMAND}
            -DINJECTOR=$<TARGET_FILE:ray_injector>
            -DCHECKER=$<TARGET_FILE:map_checker>
            -DFIXTURE=${fixture}
            -DSCALE=${fixture_scale}
            -DWORKDIR=${FIXTURE_WORKDIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_fixture.cmake)
    set_tests_properties(fixture_${fixture} PROPERTIES TIMEOUT 600)
endforeach()

# The wire path, once. Everything else replays from a recorded tlog with no
# live process, which is what keeps CI free of timing flake.
if(UNIX)
    add_test(NAME fixture_wire_smoke
        COMMAND ${CMAKE_COMMAND}
            -DINJECTOR=$<TARGET_FILE:ray_injector>
            -DCHECKER=$<TARGET_FILE:map_checker>
            -DFIXTURE=ground
            -DPORT=19488
            -DWORKDIR=${FIXTURE_WORKDIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/run_wire_fixture.cmake)
    set_tests_properties(fixture_wire_smoke PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
endif()

# The viewer must not be able to tell a synthetic ray from a lidar.
add_executable(test_no_fixture_symbols test_no_fixture_symbols.c)
target_compile_definitions(test_no_fixture_symbols PRIVATE SRC_DIR="${CMAKE_SOURCE_DIR}/src")
add_test(NAME no_fixture_symbols COMMAND test_no_fixture_symbols)
