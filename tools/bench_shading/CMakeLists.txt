set(SHADING_SRCS
    bench_shading.impala
    ${IMPALA_FILES})

anydsl_runtime_wrap(SHADING_OBJS
    NAME "bench_shading"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${SHADING_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../common/shading)

add_executable(bench_shading
    EXCLUDE_FROM_ALL# FIXME: Remove this when all errors are fixed 
    ${SHADING_OBJS}
    bench_shading.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/shading.h)
target_include_directories(bench_shading PUBLIC ../common)
target_link_libraries(bench_shading rodent_runtime ${AnyDSL_runtime_LIBRARIES})
