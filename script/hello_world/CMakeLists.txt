add_library(hello_world MODULE hello_world.cpp)

target_link_libraries(hello_world
    PRIVATE boris_common
    PRIVATE gz-sim8::gz-sim8
    PRIVATE gz-plugin2::register)
