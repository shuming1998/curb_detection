add_library(x_zero
x_zero.cpp)

target_link_libraries(x_zero
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_compile_options(x_zero PRIVATE "-march=native")
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    target_compile_options(x_zero PRIVATE -mfma)
endif ()
