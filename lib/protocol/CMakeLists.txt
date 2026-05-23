add_library(protocol
    src/protocol.c
)

target_include_directories(protocol PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(protocol
    tcp_transport
)