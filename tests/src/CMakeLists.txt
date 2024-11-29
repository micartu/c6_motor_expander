file(GLOB SRCS *.cpp ../*.cpp)
file(GLOB C_SRCS ../../Core/Src/modules/*.c adapters/*.c)

set(PROG draft_loop_control_tests)

# CppUTest
include(FetchContent)
FetchContent_Declare(
    CppUTest
    GIT_REPOSITORY https://github.com/cpputest/cpputest.git
    GIT_TAG        v4.0
)
# Set this to ON if you want to have the CppUTests in your project as well.
set(TESTS OFF CACHE BOOL "Switch off CppUTest Test build")
FetchContent_MakeAvailable(CppUTest)

add_executable(${PROG} ${SRCS} ${C_SRCS})
target_link_libraries(${PROG} PRIVATE CppUTest CppUTestExt)