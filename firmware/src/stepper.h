// Gerador de passos do DRV8825: velocidade contínua por acumulador de fase (gptimer), posição por contagem.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#define PASSOS_POR_RAD ((float)(MOTOR_PASSOS_VOLTA * MICROPASSO) * REDUCAO / 6.28318530718f)

void stepper_init(void);
void stepper_habilita(bool ligado);
bool stepper_habilitado(void);

// Velocidade do braço em rad/s (positivo = theta1 crescendo). Satura na taxa máxima do gerador.
void stepper_velocidade(float rad_s);
// Velocidade máxima que o gerador consegue (rad/s).
float stepper_v_limite(void);

// Passos contados desde o boot (sinal de theta1).
int32_t stepper_passos(void);
