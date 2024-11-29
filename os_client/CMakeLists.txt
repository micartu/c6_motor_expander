file(GLOB SRCS *.c ../Core/Src/modules/crc8.c)

set(PROG uart_client)

include_directories(../Core/Inc)
include_directories(../Core/Inc/modules)
include_directories(../tests/fakes/hal)

add_executable(${PROG} ${SRCS})
