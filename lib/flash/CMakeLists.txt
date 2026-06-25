add_library(flash
	src/flash.c
)

target_include_directories(flash PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(flash
	pico_stdlib
    hardware_flash
    hardware_sync
)