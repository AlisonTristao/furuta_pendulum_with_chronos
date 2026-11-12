#include "estimador.h"

void estimador_cria(estimador *e, double L1, double L2)
{
    e->L1 = L1, e->L2 = L2, e->iniciado = 0;
}

void estimador_passo(estimador *e, const modelo *m, double x[4], double a_ant, double Ts)
{
    if (!e->iniciado) {
        for (int i = 0; i < 4; i++) e->x[i] = x[i];
        e->iniciado = 1;
        return;
    }
    modelo_passo_acel(m, e->x, a_ant, Ts, 10);
    double inov = x[1] - e->x[1];
    e->x[0] = x[0], e->x[2] = x[2];               /* braço: o firmware sabe exatamente (passos) */
    e->x[1] += e->L1 * inov;
    e->x[3] += e->L2 * inov;
    x[3] = e->x[3];
}
