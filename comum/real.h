/* Tipo de ponto flutuante do controlador (CHRONOS, qlpv_mpc, ρ e ZOH do modelo).
 * PC: double. ESP32: float (-DREAL_FLOAT), porque o ESP32 só tem FPU de precisão simples (double é emulado,
 * ~10-20× mais lento). Com float, compile com -Werror=double-promotion: nenhuma conta do controlador pode voltar
 * para double (literal 0.5 promove: use (real)0.5). <tgmath.h>: sqrt, exp, log... no tipo do argumento. */
#ifndef REAL_H
#define REAL_H

#include <tgmath.h>

#ifdef REAL_FLOAT
typedef float real;
#define REAL_MIN_PASSO 1e-30f   /* menor passo da busca de linha */
#else
typedef double real;
#define REAL_MIN_PASSO 1e-300
#endif

#endif
