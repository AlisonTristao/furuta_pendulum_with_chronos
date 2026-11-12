/* Controlador completo de um período, igual no ESP32 (embarcado) e no sim.exe (ensaio.embarcado = 1):
 * estimador de dθ2, compensação do atraso, NMPC qLPV e u -> θ̈1.
 *
 * Atraso fixo de um período: o θ̈1 calculado com a amostra k é aplicado no instante k+1 (até k+2). Então, na
 * amostra k: o estimador usa o comando que valeu em [k-1, k] (calculado em k-2), a compensação prediz o estado
 * comp_atraso_ms à frente com o comando que vale em [k, k+1] (calculado em k-1; comp_atraso_ms = Ts é exato). */
#ifndef CONTROLADOR_H
#define CONTROLADOR_H

#include "config.h"
#include "estimador.h"

typedef struct {
    config cfg;           /* cópia (θ1_min/max já com o limite do homing) */
    modelo m;
    mpc *c;
    estimador est;
    double a_ap, a_int;   /* θ̈1 aplicado agora (calculado no passo anterior) e no intervalo que terminou */
    double off2;          /* múltiplo de 2π tirado de θ2 na partida */
    double u;             /* último torque do MPC */
} controlador;

/* x0: medida na partida (θ2 do firmware: π = embaixo). Devolve 0 se ok. */
int controlador_cria(controlador *k, const config *cfg, const double x0[4]);
void controlador_libera(controlador *k);
/* Um período. x: medida da amostra k; sai como o controle viu (θ2 sem o múltiplo de 2π, dθ2 estimado).
 * t: tempo desde a partida (referência de θ1). Devolve o θ̈1 para aplicar no instante k+1. */
double controlador_passo(controlador *k, double x[4], double t, mpc_info *in);

#endif
