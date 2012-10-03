project(cmake-qt)
cmake_minimum_required(VERSION 2.8)

#aux_source_directory(. SRC_LIST)

file(GLOB SRC_LIST "./*.c**")



include_directories(/usr/msp430/include)
LINK_DIRECTORIES(/usr/msp430/lib)

set(CMAKE_C_COMPILER   /usr/bin/msp430-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/msp430-gcc)

set(CMAKE_C_LINK_EXECUTABLE   /usr/bin/msp430-gcc)
set(CMAKE_CXX_LINK_EXECUTABLE /usr/bin/msp430-gcc)

SET(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_CXX_COMPILER> -pipe -Os <FLAGS> -mmcu=msp430g2553 <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")

set(CMAKE_C_FLAGS "-pipe -Os -mmcu=msp430g2553")

#set(CMAKE_EXE_LINKER_FLAGS " -mmcu=msp430g2553")
#set(CMAKE_CXX_LINKER_FLAGS " -mmcu=msp430g2553")
#set(LFLAGS "-mmcu=msp430g2553")


add_executable(${PROJECT_NAME} ${SRC_LIST})
