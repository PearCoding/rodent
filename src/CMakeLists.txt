set(CPUINFO_PATH "/proc/cpuinfo" CACHE STRING "Path to the CPU info file on the system")
set(TARGET_PLATFORM "" CACHE STRING "Target platform for the converter tool. Leave empty to autodetect CPU. Use converter --help to list possible platforms.")
set(TARGET_DEVICE "" CACHE STRING "Target device on the selected platform. Leave empty to use the default.")
set(SCENE_FILE "${PROJECT_SOURCE_DIR}/testing/cornell_box.obj" CACHE FILEPATH "Absolute path to an OBJ scene")
set(MEGAKERNEL_FUSION ON CACHE BOOL "Set to true to enable simple shader fusion for megakernel mappings")
set(MAX_PATH_LEN "64" CACHE STRING "Maximum path length")
set(DISABLE_GUI OFF CACHE BOOL "Set to true to disable GUI")
set(SPP "4" CACHE STRING "Samples per pixel")
if (SCENE_FILE STREQUAL "")
    message(FATAL_ERROR "Please specify a valid OBJ scene in the SCENE_FILE variable")
endif()
set(GENERATOR_OPTIONS "")
if (NOT TARGET_PLATFORM STREQUAL "")
    set(GENERATOR_OPTIONS "--target" "${TARGET_PLATFORM}")
endif()
if (NOT TARGET_DEVICE STREQUAL "")
    set(GENERATOR_OPTIONS ${GENERATOR_OPTIONS} "--device" "${TARGET_DEVICE}")
    if (MEGAKERNEL_FUSION AND (TARGET_PLATFORM STREQUAL "nvvm-megakernel" OR TARGET_PLATFORM STREQUAL "amdgpu-megakernel"))
        set(GENERATOR_OPTIONS ${GENERATOR_OPTIONS} "--fusion")
    endif()
endif()

set(RODENT_SRCS
    impala/core/color.impala
    impala/core/common.impala
    impala/core/cpu_common.impala
    impala/core/illumination.impala
    impala/core/matrix.impala
    impala/core/random.impala
    impala/core/sort.impala
    impala/core/spectral.impala
    impala/core/vector.impala
    impala/render/image.impala
    impala/render/camera.impala
    impala/render/geometry.impala
    impala/render/light.impala
    impala/render/material.impala
    impala/render/renderer.impala
    impala/render/scene.impala
    impala/render/driver.impala
    impala/render/mapping_cpu.impala
    impala/render/mapping_gpu.impala
    impala/render/tonemapper.impala
    impala/traversal/intersection.impala
    impala/traversal/stack.impala
    impala/traversal/mapping_cpu.impala
    impala/traversal/mapping_gpu.impala)

list(TRANSFORM RODENT_SRCS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/" OUTPUT_VARIABLE IMPALA_FILES)
set(IMPALA_FILES ${IMPALA_FILES} PARENT_SCOPE)

set(DRIVER_SRCS
    driver/driver.cpp
    driver/interface.cpp
    driver/interface.h)

set(GENERATOR_SRCS
    generator/bvh.h
    generator/convert_mts.h
    generator/convert_mts.cpp
    generator/convert_obj.h
    generator/convert_obj.cpp
    generator/export_image.h
    generator/export_image.cpp
    generator/generator.cpp
    generator/impala.h
    generator/platform.h
    generator/spectral.h
    generator/spectral.cpp
    generator/target.h)

set(RUNTIME_SRCS
    runtime/obj.cpp
    runtime/obj.h
    runtime/file_path.h
    runtime/image_exr.cpp
    runtime/image_jpg.cpp
    runtime/image_png.cpp
    runtime/image_utils.cpp
    runtime/image.h
    runtime/bvh.h
    runtime/common.h
    runtime/color.h
    runtime/float2.h
    runtime/float3.h
    runtime/float4.h
    runtime/tri.h
    runtime/bbox.h
    runtime/buffer.h
)

anydsl_runtime_wrap(RODENT_OBJS
    NAME "rodent"
    CLANG_FLAGS ${CLANG_FLAGS}
    IMPALA_FLAGS --log-level info
    FILES ${RODENT_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/main.impala)

anydsl_runtime_wrap(DISCARD_TMP_OBJS
    NAME "interface"
    FILES ${RODENT_SRCS} impala/dummy_main.impala
    INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/driver/interface)

if (NOT DISABLE_GUI)
    find_package(SDL2 REQUIRED)
endif()
find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TBB REQUIRED)
find_package(LZ4 REQUIRED)

add_library(rodent_runtime STATIC ${RUNTIME_SRCS})
target_include_directories(rodent_runtime PUBLIC ${LZ4_INCLUDE_DIR} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_include_directories(rodent_runtime PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/external/tinyexr>)
target_link_libraries(rodent_runtime PUBLIC ${LZ4_LIBRARY} ${PNG_LIBRARIES} ${JPEG_LIBRARIES})

add_executable(rodent_generator ${GENERATOR_SRCS})
target_include_directories(rodent_generator PUBLIC ${LZ4_INCLUDE_DIR} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(rodent_generator PUBLIC rodent_runtime PRIVATE tinyparser_mitsuba)
target_compile_definitions(rodent_generator PUBLIC -DCPUINFO_PATH="${CPUINFO_PATH}")
if (COLORIZE)
    target_compile_definitions(rodent_generator PUBLIC -DCOLORIZE)
endif()
if (EMBREE_FOUND)
    target_include_directories(rodent_generator PUBLIC ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR} ${EMBREE_LIBRARY_DIR})
    target_link_libraries(rodent_generator ${EMBREE_DEPENDENCIES})
    target_compile_definitions(rodent_generator PUBLIC ${EMBREE_DEFINITIONS} -DENABLE_EMBREE_BVH)
endif()

# Add simple (possible) dependency to the material file
string(REGEX REPLACE "[.]obj$" ".mtl" SCENE_FILE_MTL ${SCENE_FILE})
set(GENERATOR_DEPENDENCIES ${SCENE_FILE})
if(EXISTS ${SCENE_FILE_MTL})
    list(APPEND GENERATOR_DEPENDENCIES ${SCENE_FILE_MTL})
endif()

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/main.impala
    COMMAND ${CMAKE_COMMAND} -E copy ../tools/upsampler/srgb.coeff ${CMAKE_BINARY_DIR}/srgb.coeff 
    COMMAND rodent_generator ${SCENE_FILE} ${GENERATOR_OPTIONS} --max-path-len ${MAX_PATH_LEN} --samples-per-pixel ${SPP}
    COMMAND ${CMAKE_COMMAND} -E rename ${CMAKE_BINARY_DIR}/main.impala ${CMAKE_CURRENT_BINARY_DIR}/main.impala
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    DEPENDS ${GENERATOR_DEPENDENCIES} rodent_generator)

add_custom_target(convert DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/main.impala)

set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/main.impala PROPERTIES GENERATED TRUE)

add_library(rodent_driver STATIC ${DRIVER_SRCS})
target_link_libraries(rodent_driver PUBLIC rodent_runtime)
target_include_directories(rodent_driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
if (EMBREE_FOUND)
    target_include_directories(rodent_driver PUBLIC ${EMBREE_ROOT_DIR}/include ${EMBREE_ROOT_DIR} ${EMBREE_LIBRARY_DIR} ${PNG_INCLUDE_DIRS} ${JPEG_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${LZ4_INCLUDE_DIR})
    target_link_libraries(rodent_driver PUBLIC ${EMBREE_DEPENDENCIES})
    target_compile_definitions(rodent_driver PUBLIC -DENABLE_EMBREE_DEVICE)
endif()
if (NOT DISABLE_GUI)
    target_include_directories(rodent_driver PUBLIC ${SDL2_INCLUDE_DIR})
    target_link_libraries(rodent_driver PUBLIC ${SDL2_LIBRARY})
else()
    target_compile_definitions(rodent_driver PUBLIC -DDISABLE_GUI)
endif()
if (COLORIZE)
    target_compile_definitions(rodent_driver PUBLIC -DCOLORIZE)
endif()

add_executable(rodent ${RODENT_OBJS})
target_include_directories(rodent PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(rodent PUBLIC rodent_driver ${AnyDSL_runtime_LIBRARIES} ${TBB_LIBRARIES})

if (SCENE_FILE STREQUAL "${PROJECT_SOURCE_DIR}/testing/cornell_box.obj")
    # Test rodent when the cornell box is used
    add_test(NAME rodent_cornell COMMAND ${CMAKE_COMMAND} -DRODENT=$<TARGET_FILE:rodent> -DIM_COMPARE=${ImageMagick_compare_EXECUTABLE} "-DRODENT_ARGS=--eye;0;1;2.7;--dir;0;0;-1;--up;0;1;0" -DTESTING_DIR=${PROJECT_SOURCE_DIR}/testing -DRODENT_DIR=${CMAKE_BINARY_DIR} -DRODENT_OUTPUT=rodent-cornell-output -P ${PROJECT_SOURCE_DIR}/cmake/test/run_rodent.cmake)
endif()
