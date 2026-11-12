# Núcleo do pêndulo em C: malha de 1 kHz, motor de passo, encoder, homing, controle embarcado (MPC no núcleo 1),
# benchmark e teste de pinos. Não conhece o BTP: o texto sai por saida.c, que o src/main.cpp liga ao TraceView.

# pc/furuta.ini embutido (furuta_ini.c, gerado aqui a cada configuração do CMake): é a configuração padrão do
# controle no boot (embarcado.c), a mesma do simulador. O PlatformIO não roda o EMBED_TXTFILES do IDF.
set(INI ${CMAKE_CURRENT_LIST_DIR}/../../../pc/furuta.ini)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${INI})
file(READ ${INI} _ini HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _ini "${_ini}")
file(WRITE ${CMAKE_CURRENT_LIST_DIR}/furuta_ini.c
     "/* gerado pelo CMakeLists.txt a partir de pc/furuta.ini: não editar */\n"
     "const char furuta_ini[] = {${_ini} 0};\n")

idf_component_register(SRCS bench.c controle.c embarcado.c encoder.c furuta_ini.c homing.c ident.c pinos.c saida.c stepper.c
                       INCLUDE_DIRS "."
                       REQUIRES mpc esp_driver_gpio
                       PRIV_REQUIRES esp_driver_gptimer esp_driver_i2c esp_timer esp_rom esp_hw_support)
