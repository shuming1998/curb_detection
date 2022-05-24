add_library(star_shaped
star_shaped.cpp)

target_link_libraries(star_shaped
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_compile_options(star_shaped PRIVATE "-march=native")
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    target_compile_options(star_shaped PRIVATE -mfma)
endif ()
