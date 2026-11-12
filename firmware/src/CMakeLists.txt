# Só C++: o dispositivo BTP (comandos, tópicos) e o app_main. O núcleo em C está em components/furuta.
idf_component_register(SRCS main.cpp
                       PRIV_REQUIRES furuta BTPDevice)
