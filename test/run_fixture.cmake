# Drives one fixture end to end: record a tlog, then score it.
#
# CI never runs a live process -- the injector writes the tlog and the ground
# truth, and the checker replays that. The wire path has its own test.

if(NOT DEFINED INJECTOR OR NOT DEFINED CHECKER OR NOT DEFINED FIXTURE OR NOT DEFINED WORKDIR)
    message(FATAL_ERROR "run_fixture.cmake needs INJECTOR, CHECKER, FIXTURE and WORKDIR")
endif()

if(NOT DEFINED SCALE)
    set(SCALE 1)
endif()
if(NOT DEFINED SEED)
    set(SEED 1)
endif()

file(MAKE_DIRECTORY "${WORKDIR}")
set(TLOG "${WORKDIR}/${FIXTURE}.tlog")
set(TRUTH "${WORKDIR}/${FIXTURE}.truth")

set(SPLAT_ARGS)
if(DEFINED SPLAT AND NOT SPLAT STREQUAL "")
    set(SPLAT_ARGS --splat "${SPLAT}")
endif()

execute_process(
    COMMAND "${INJECTOR}" --fixture "${FIXTURE}" --tlog "${TLOG}" --truth "${TRUTH}"
            --seed "${SEED}" --scale "${SCALE}" ${SPLAT_ARGS}
    RESULT_VARIABLE inject_rc
    OUTPUT_VARIABLE inject_out
    ERROR_VARIABLE  inject_err)
message(STATUS "${inject_out}")
if(NOT inject_rc EQUAL 0)
    message(FATAL_ERROR "injector failed (${inject_rc}): ${inject_err}")
endif()

execute_process(
    COMMAND "${CHECKER}" --truth "${TRUTH}" --tlog "${TLOG}"
    RESULT_VARIABLE check_rc
    OUTPUT_VARIABLE check_out
    ERROR_VARIABLE  check_err)
message("${check_out}")
if(NOT check_rc EQUAL 0)
    message(FATAL_ERROR "fixture '${FIXTURE}' failed its thresholds: ${check_err}")
endif()
