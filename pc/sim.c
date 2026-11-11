/* Malha fechada simulada: NMPC qLPV (CHRONOS em C) + planta não linear (RK4). Mesmo controlador do ensaio real.
 *
 *   sim.exe <config.ini> [saida.csv]
 *
 * CSV: t, θ1, θ2, dθ1, dθ2, θ1_ref, u (torque), a (θ̈1 que o motor imporia), n_qp, n_it, dP, t_calc_ms */
#include "config.h"
#include "modelo.h"
#include "periodo.h"
#include "qlpv_mpc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846
static double wrap(double a) { return a - 2 * PI * floor((a + PI) / (2 * PI)); }

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "uso: %s <config.ini> [saida.csv]\n", argv[0]);
        return 2;
    }
    config cfg;
    if (config_le(argv[1], &cfg)) return 1;
    config_imprime(&cfg);
    const ensaio *e = &cfg.e;
    modelo m = modelo_de_fisico(cfg.f, e->Ts, cfg.mpc.dtheta_max);
    double x[4] = {e->x0[0], e->x0[1], e->x0[2], e->x0[3]};
    mpc *c = mpc_cria(&cfg.mpc, &m, x, 0.0);
    FILE *out = argc > 2 ? fopen(argv[2], "w") : NULL;
    if (out) fprintf(out, "t,theta1,theta2,dtheta1,dtheta2,theta1_ref,u,a,n_qp,n_it,dP,t_calc_ms\n");

    int K = (int)lround(e->duracao / e->Ts);
    double t_up = -1, soma_tc = 0, max_tc = 0, soma_qp = 0, max_a = 0;
    int k;
    for (k = 0; k < K; k++) {
        double t = k * e->Ts;
        double ref[4] = {t >= e->t_degrau ? e->theta1_degrau : e->theta1_ref, 0, 0, 0};
        mpc_info in;
        double t0 = periodo_agora();
        double u = mpc_passo(c, x, ref, &in);
        double tc = (periodo_agora() - t0) * 1e3;
        double a = modelo_acel_braco(&m, x, u);
        soma_tc += tc;
        max_tc = fmax(max_tc, tc);
        soma_qp += in.n_qp;
        max_a = fmax(max_a, fabs(a));
        if (out)
            fprintf(out, "%.4f,%.10g,%.10g,%.10g,%.10g,%.6g,%.12g,%.8g,%d,%d,%.3e,%.4f\n", t, x[0], x[1], x[2], x[3],
                    ref[0], u, a, in.n_qp, in.n_it, in.dP, tc);
        if (t_up < 0 && fabs(wrap(x[1])) < 0.1 && fabs(x[3]) < 0.5) t_up = t;
        if (!in.finito) {
            printf("controle não finito em t = %.2f s\n", t);
            break;
        }
        double ud = u + (t >= e->t_disturbio && t < e->t_disturbio + e->dur_disturbio ? e->u_disturbio : 0);
        modelo_passo(&m, x, ud, e->rk4_sub);
    }
    printf("passos %d | swing-up (|θ2| < 0,1 rad) em %.2f s | x final = (%.3f, %.3f, %.3f, %.3f)\n", k, t_up, x[0],
           x[1], x[2], x[3]);
    printf("t_calc: médio %.2f ms, máx %.2f ms | QPs por passo: %.2f | máx |θ̈1| = %.1f rad/s²\n", soma_tc / (k ? k : 1), max_tc,
           soma_qp / (k ? k : 1), max_a);
    if (out) fclose(out);
    mpc_libera(c);
    return 0;
}
