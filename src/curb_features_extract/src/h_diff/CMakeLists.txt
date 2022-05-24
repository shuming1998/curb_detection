add_library(h_diff
h_diff.cpp)

target_link_libraries(h_diff
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_compile_options(h_diff PRIVATE "-march=native")
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    target_compile_options(h_diff PRIVATE -mfma)
endif ()
