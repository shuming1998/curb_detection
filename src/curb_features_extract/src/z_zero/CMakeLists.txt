add_library(z_zero
z_zero.cpp)

target_link_libraries(z_zero
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_compile_options(z_zero PRIVATE "-march=native")
if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
    target_compile_options(z_zero PRIVATE -mfma)
endif ()
