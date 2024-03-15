
set(carbase_SRC
    can.cpp
    carbase.cpp
    carbase_node.cpp
)

add_executable(carbase ${carbase_SRC})
target_link_libraries(carbase rb_common ${catkin_LIBRARIES})
