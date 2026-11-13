# Controlador compartilhado com o PC (comum/ na raiz do repositório), em float: o ESP32 só tem FPU de precisão
# simples. Otimização própria (o resto do firmware segue o sdkconfig): MPC_OPT=-O3 no ambiente troca
# (padrão -O3, 13% mais rápido que -O2 no bench; faça pio run -t clean antes de trocar).
set(COMUM ${CMAKE_CURRENT_LIST_DIR}/../../../comum)
idf_component_register(SRCS ${COMUM}/chronos.c ${COMUM}/qlpv_mpc.c ${COMUM}/modelo.c ${COMUM}/estimador.c
                            ${COMUM}/config.c ${COMUM}/controlador.c ${COMUM}/planta.c
                       INCLUDE_DIRS ${COMUM}
                       LDFRAGMENTS linker.lf)   # tudo na RAM (linker.lf)
target_compile_definitions(${COMPONENT_LIB} PUBLIC REAL_FLOAT CHRONOS_FIXO)   # FIXO: tamanhos constantes (chronos.c)
set(MPC_OPT "$ENV{MPC_OPT}")
if(NOT MPC_OPT)
    set(MPC_OPT "-O3")
endif()
target_compile_options(${COMPONENT_LIB} PRIVATE ${MPC_OPT} -Werror=double-promotion)
target_compile_definitions(${COMPONENT_LIB} PUBLIC MPC_OPT_STR="${MPC_OPT}")
# perfil de tempo por etapa (furuta -bench): sempre compilado, ligado só durante o bench (chronos_relogio != NULL)
target_compile_definitions(${COMPONENT_LIB} PUBLIC CHRONOS_PERFIL)
message(STATUS "controlador (componente mpc) compilado com ${MPC_OPT}")
