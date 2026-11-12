#include "controlador.h"

#define PI_D 3.14159265358979323846

int controlador_cria(controlador *k, const config *cfg, const double x0[4])
{
    k->cfg = *cfg;
    const ensaio *e = &k->cfg.e;
    k->m = modelo_de_fisico(k->cfg.f, e->Ts, k->cfg.mpc.dtheta_max);
    /* θ2 no múltiplo de 2π mais próximo do topo (equilíbrio) ou de embaixo (swing-up); depois fica contínuo */
    k->off2 = e->equilibrio ? 2 * PI_D * floor((x0[1] + PI_D) / (2 * PI_D)) : 2 * PI_D * floor(x0[1] / (2 * PI_D));
    double x[4] = {x0[0], x0[1] - k->off2, x0[2], x0[3]};
    k->c = mpc_cria(&k->cfg.mpc, &k->m, x, 0.0);
    if (!k->c) return -1;
    estimador_cria(&k->est, e->est_L1, e->est_L2);
    k->a_ap = k->a_int = 0;
    k->u = 0;
    return 0;
}

void controlador_libera(controlador *k)
{
    mpc_libera(k->c);
    k->c = 0;
}

double controlador_passo(controlador *k, double x[4], double t, mpc_info *in)
{
    const ensaio *e = &k->cfg.e;
    x[1] -= k->off2;
    if (e->estimador) estimador_passo(&k->est, &k->m, x, k->a_int, e->Ts);   /* x[3] passa a ser o estimado */
    double xc[4] = {x[0], x[1], x[2], x[3]};
    if (e->comp_atraso_ms > 0) modelo_passo_acel(&k->m, xc, k->a_ap, e->comp_atraso_ms * 1e-3, 4);
    double ref[4] = {t >= e->t_degrau ? e->theta1_degrau : e->theta1_ref, 0, 0, 0};
    double u = mpc_passo(k->c, xc, ref, in);
    double a = in->finito ? modelo_acel_braco(&k->m, xc, u) : 0.0;
    k->u = u;
    k->a_int = k->a_ap;
    k->a_ap = a;
    return a;
}
