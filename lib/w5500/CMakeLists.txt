add_library(w5500
    src/w5500.c
)

target_include_directories(w5500 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

pico_generate_pio_header(w5500
    ${CMAKE_CURRENT_SOURCE_DIR}/include/w5500_spi.pio
)

target_link_libraries(w5500
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_dma
    hardware_irq
    hardware_clocks
)