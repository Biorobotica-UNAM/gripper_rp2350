add_library(motor_controller
	src/motor_controller.cpp
)

target_include_directories(motor_controller PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(motor_controller
	pico_stdlib
	hardware_pwm
	hardware_gpio
)