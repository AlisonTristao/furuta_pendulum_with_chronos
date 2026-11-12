# Só C++: o dispositivo BTP (comandos, tópicos), a gravação sem botões e o app_main. O núcleo em C está em
# components/furuta.
idf_component_register(SRCS main.cpp gravacao.cpp
                       PRIV_REQUIRES furuta BTPDevice espressif__esp_tinyusb esp_system)
