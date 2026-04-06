add_library(tcp_ip
	src/tcp_ip.c
)

target_include_directories(tcp_ip PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(tcp_ip
	pico_stdlib
	w5500
)