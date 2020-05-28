set(INTERFACE_SRCS
    bench_interface.impala
    ${IMPALA_FILES})

anydsl_runtime_wrap(INTERFACE_OBJS
    NAME "bench_interface"
    CLANG_FLAGS ${CLANG_FLAGS}
    FILES ${INTERFACE_SRCS}
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../common/interface)

add_executable(bench_interface
    ${INTERFACE_OBJS}
    bench_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/interface.h)
target_include_directories(bench_interface PUBLIC ../common)
target_link_libraries(bench_interface rodent_runtime ${AnyDSL_runtime_LIBRARIES})
