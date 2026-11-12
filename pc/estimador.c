#include "estimador.h"

#include <math.h>

void estimador_cria(estimador *e, double L1, double L2)
{
    e->L1 = L1, e->L2 = L2, e->iniciado = 0;
    e->janela = 0;
}

void estimador_topo(estimador *e, double L1t, double L2t, double janela)
{
    e->L1t = L1t, e->L2t = L2t, e->janela = janela;
}

void estimador_passo(estimador *e, const modelo *m, double x[4], double a_ant, double Ts)
{
    if (!e->iniciado) {
        for (int i = 0; i < 4; i++) e->x[i] = x[i];
        e->iniciado = 1;
        return;
    }
    modelo_passo_acel(m, e->x, a_ant, Ts, 10);   /* predição */
    double inov = x[1] - e->x[1];
    e->x[0] = x[0], e->x[2] = x[2];               /* braço: o firmware sabe exatamente (passos) */
    double th = e->x[1] - 6.283185307179586 * floor((e->x[1] + 3.141592653589793) / 6.283185307179586);
    int topo = e->janela > 0 && fabs(th) < e->janela;
    e->x[1] += (topo ? e->L1t : e->L1) * inov;
    e->x[3] += (topo ? e->L2t : e->L2) * inov;
    x[3] = e->x[3];
}
