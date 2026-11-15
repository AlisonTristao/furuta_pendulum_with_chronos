// Inclinação da mesa pela varredura do braço (inclina.c): furuta -inclina, e o -auto entre o homing e o start.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Precisa do homing (malha em READY) e do pêndulo pendurado. varredura = true (furuta -inclina): a mesa inteira
// (inclinacao.pontos se >= 4, senão 7; ~1 min), acerta o zero do θ2 e grava seno/cosseno na configuração em uso e na
// flash (NVS, vale depois do boot). false (-auto): inclinacao.pontos (1 = só o zero em θ1 = 0, ~8 s). Imprime no log.
// NULL se deu certo, ou o motivo.
const char *inclina_roda(bool varredura);

#ifdef __cplusplus
}
#endif
