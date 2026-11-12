// Controle embarcado: o NMPC qLPV (comum/controlador.c, float) roda no núcleo 1, sincronizado com a malha de
// 1 kHz (controle.c, núcleo 0). Atraso fixo de um período: o θ̈1 calculado com a amostra k é aplicado na
// amostra k+1. Telemetria a cada período pela serial ("T,...") por uma tarefa de baixa prioridade no núcleo 0.
#pragma once

#include <stdbool.h>

void embarcado_init(void);

// "secao.chave=valor" (as mesmas chaves do pc/furuta.ini). false se a chave não existir ou o valor for inválido.
bool embarcado_config(const char *par);

// Liga: modo 1 = controle (precisa de READY após o homing; espera a condição de partida do ensaio e então
// controla por ensaio.duracao); modo 2 = a seco (calcula com os sensores, mas não comanda o motor).
// Devolve NULL se ligou, ou o motivo da recusa.
const char *embarcado_liga(int modo);
void embarcado_desliga(void);
bool embarcado_ativo(void);
