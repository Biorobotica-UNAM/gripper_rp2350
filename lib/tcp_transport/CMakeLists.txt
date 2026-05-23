add_library(tcp_transport
	src/tcp_transport.c
)

target_include_directories(tcp_transport PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(tcp_transport
	pico_stdlib
	pico_lwip
	w5500
)