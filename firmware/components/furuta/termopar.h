// Temperatura do driver por um termopar tipo K num módulo MAX6675 (termopar.c). O MAX6675 é SPI só de leitura
// (SCK, CS, SO; pinos em config.h), lido a cada 250 ms numa tarefa própria (ele converte em ~220 ms).
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void termopar_init(void);          // configura os pinos e cria a tarefa (núcleo 0, prioridade baixa)
// Última temperatura [°C], 0,25 °C de resolução. NAN se não há leitura válida (termopar_estado diz por quê).
float termopar_temp(void);
const char *termopar_estado(void);  // "ok", "termopar aberto", "sem módulo" ou "iniciando"

#ifdef __cplusplus
}
#endif
