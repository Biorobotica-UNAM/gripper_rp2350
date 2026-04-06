add_library(quadrature_encoder
	src/quadrature_encoder.cpp
)

target_include_directories(quadrature_encoder PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

pico_generate_pio_header(quadrature_encoder
	${CMAKE_CURRENT_SOURCE_DIR}/include/quadrature_encoder.pio
)

target_link_libraries(quadrature_encoder
	pico_stdlib
	hardware_pio
	hardware_dma
	hardware_gpio
	hardware_sync
)