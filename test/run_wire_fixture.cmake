# Wire smoke test: the injector emits over UDP, exactly as a vehicle does, and
# the checker binds a real socket to receive it. This is the test that proves
# the fixtures reach the map through the ingest path rather than around it.
#
# The checker binds first and then spawns the injector itself, so nothing is
# transmitted into a closed port.

if(NOT DEFINED INJECTOR OR NOT DEFINED CHECKER OR NOT DEFINED FIXTURE
   OR NOT DEFINED WORKDIR OR NOT DEFINED PORT)
    message(FATAL_ERROR "run_wire_fixture.cmake needs INJECTOR, CHECKER, FIXTURE, WORKDIR and PORT")
endif()

file(MAKE_DIRECTORY "${WORKDIR}")
set(TRUTH "${WORKDIR}/${FIXTURE}-wire.truth")

# A silent pass first, so the checker knows the geometry and thresholds before
# anything is transmitted.
execute_process(
    COMMAND "${INJECTOR}" --fixture "${FIXTURE}" --truth "${TRUTH}" --seed 1
    RESULT_VARIABLE prep_rc
    OUTPUT_QUIET ERROR_VARIABLE prep_err)
if(NOT prep_rc EQUAL 0)
    message(FATAL_ERROR "truth generation failed: ${prep_err}")
endif()

execute_process(
    COMMAND "${CHECKER}" --truth "${TRUTH}" --listen "${PORT}"
            --spawn "${INJECTOR}" --fixture "${FIXTURE}" --seconds 40
    RESULT_VARIABLE rc
    OUTPUT_VARIABLE out
    ERROR_VARIABLE  err
    TIMEOUT 90)
message("${out}")
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "wire smoke test failed (${rc}): ${err}")
endif()
