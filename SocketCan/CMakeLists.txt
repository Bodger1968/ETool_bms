find_package(Threads REQUIRED)

add_library(socketcan STATIC
    src/SocketCAN.cpp
        src/SocketCANRaw.cpp)

target_include_directories(socketcan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/headers)

target_link_libraries(socketcan PUBLIC Threads::Threads)