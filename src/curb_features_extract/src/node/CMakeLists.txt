add_executable(test_star_shaped_node
test_star_shaped_node.cpp
)

target_link_libraries(test_star_shaped_node
    star_shaped
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(test_x_zero_node
test_x_zero_node.cpp
)

target_link_libraries(test_x_zero_node
    x_zero
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(test_z_zero_node
test_z_zero_node.cpp
)

target_link_libraries(test_z_zero_node
    z_zero
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(test_h_diff_node
test_h_diff_node.cpp
)

target_link_libraries(test_h_diff_node
    h_diff
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
)

