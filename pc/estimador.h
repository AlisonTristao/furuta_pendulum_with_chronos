/* Estimador de (θ2, dθ2) no PC: a cada Ts integra o modelo com a aceleração do braço que foi comandada (conhecida)
 * e corrige com o θ2 medido (AS5600, 12 bits). Substitui a velocidade do observador do firmware, que atrasa ~8 ms
 * (erro rms ~0,95 rad/s no topo nos ensaios de 10/11/2026; este estimador: ~0,3 rad/s nos mesmos dados). */
#ifndef ESTIMADOR_H
#define ESTIMADOR_H

#include "modelo.h"

typedef struct {
    double x[4];       /* estado estimado */
    double L1, L2;     /* ganhos da correção em θ2 e dθ2 */
    int iniciado;
} estimador;

void estimador_cria(estimador *e, double L1, double L2);
/* x: medida (θ1, θ2, dθ1, dθ2 do firmware) -> sobrescreve x[3] (e não mexe no resto). a_ant: θ̈1 comandada no
 * intervalo que terminou. Ts: período. */
void estimador_passo(estimador *e, const modelo *m, double x[4], double a_ant, double Ts);

#endif
