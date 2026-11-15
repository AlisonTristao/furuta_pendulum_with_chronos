// Malha interna de 1 kHz: lê encoder e fins de curso, integra a aceleração pedida em velocidade de passo,
// cuida do homing, dos limites de software, do watchdog e das falhas.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum { EST_OCIOSO, EST_HOMING, EST_PRONTO, EST_RODANDO, EST_FALHA } estado_t;

typedef struct {
    float t1, t2, dt1, dt2;   // theta1 (braço), theta2 (pêndulo, 0 = topo) e velocidades
} leitura_t;

void controle_init(void);     // cria a tarefa (núcleo 0) e espera o hardware subir

estado_t controle_estado(void);
const char *controle_estado_nome(estado_t e);
const char *controle_erro(void);
void controle_leitura(leitura_t *x);
bool controle_fim_neg(void);
bool controle_fim_pos(void);
bool controle_falha_driver(void);

// Comando da malha externa: aceleração do braço (rad/s²). Só aceito em PRONTO/RODANDO.
bool controle_acel(float a);
bool controle_homing(void);   // inicia o homing (liga o driver); false se não for possível agora
// Movimentos feitos pela própria malha, a partir de READY (false se não estiver em READY). Um comando de
// controle_acel ou o homing cancelam. controle_movendo() fica true até acabar.
bool controle_vai(float alvo);                 // até theta1 = alvo, perfil trapezoidal do homing
// θ̈1 = A cos(w t) por `ciclos` ciclos (v = A/w sin(w t)): na frequência do pêndulo a oscilação dele cresce a cada
// ciclo; o braço anda até 2A/w² e volta parado ao mesmo lugar
bool controle_excita(float A, float w, int ciclos);
bool controle_movendo(void);
// Repouso: true corta a corrente do motor (EN do DRV8825) sem perder a posição do indexador nem os passos contados;
// false religa. Só vale em READY/RUN; o braço fica solto enquanto isso (empurrado à mão, o θ1 contado erra).
void controle_solta_braco(bool solto);
// Depois de um homing: centro novo − centro do homing anterior [passos] = passos que o motor perdeu entre os dois
// (positivo: o braço andou menos no sentido negativo do que o contado). *solto: o driver desligou no meio (o braço
// pode ter sido mexido à mão). false no primeiro homing ou depois de um furuta -origem.
bool controle_desvio_homing(int32_t *passos, bool *solto);
void controle_origem_aqui(void);   // theta1 = 0 na posição atual, limite ±LIM_SEM_HOMING_RAD (bancada)
void controle_zera_pendulo(void);  // pêndulo parado embaixo: theta2 = pi
void controle_desloca_pendulo(float delta);   // theta2 += delta (zero pela média, embarcado.c)
void controle_habilita(bool ligado);
float controle_meio_curso(void);
float controle_limite(void);       // limite de software de |theta1|

// Controle embarcado: a cada periodo_ms a malha guarda a leitura daquele instante e notifica `tarefa`
// (xTaskNotifyGive); NULL cancela. controle_amostra devolve a última leitura guardada.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
void controle_assina_amostra(TaskHandle_t tarefa, int periodo_ms);
void controle_amostra(leitura_t *x);
