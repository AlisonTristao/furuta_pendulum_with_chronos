file(GLOB app_sources "*.c")

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES mpc esp_driver_gpio esp_driver_gptimer esp_driver_i2c esp_driver_uart esp_timer)
