set(TRAVERSAL_SRCS
    bench_traversal.impala
    ${IMPALA_FILES})

anydsl_runtime_wrap(TRAVERSAL_OBJS
    NAME "bench_traversal"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${TRAVERSAL_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../common/traversal)

add_executable(bench_traversal
    ${TRAVERSAL_OBJS}
    bench_traversal.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/traversal.h)
target_include_directories(bench_traversal PUBLIC ../common)
target_link_libraries(bench_traversal ${AnyDSL_runtime_LIBRARIES})
if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc)
    add_custom_command(TARGET bench_traversal POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/bench_traversal.nvvm.bc ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()
