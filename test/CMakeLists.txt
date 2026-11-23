cmake_minimum_required(VERSION 3.15)
project(stm_self_balancing_robot_tests C)
enable_testing()
add_executable(test_crsf test_crsf.c
    ../Core/Src/crsf.c
    unity/src/unity.c)
target_compile_options(test_crsf PRIVATE
    -fsanitize=address
    -g)
target_link_options(test_crsf PRIVATE
    -fsanitize=address)
target_include_directories(test_crsf PRIVATE
    ../Core/Inc
    unity/src)
add_test(NAME crsf COMMAND test_crsf)