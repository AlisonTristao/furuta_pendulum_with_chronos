/* Malha fechada simulada: NMPC qLPV (CHRONOS em C) + planta não linear (RK4). Mesmo controlador do ensaio real.
 *
 *   sim.exe <config.ini> [saida.csv]
 *
 * CSV: t, θ1, θ2, dθ1, dθ2 (o que o controle viu, como no malha.c), θ1_ref, u (torque), a (θ̈1 comandado), n_qp, n_it,
 *      dP, t_calc_ms, eq, e o estado verdadeiro: theta1_real, theta2_real, dtheta2_real */
#include "config.h"
#include "estimador.h"
#include "modelo.h"
#include "periodo.h"
#include "qlpv_mpc.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include <windows.h>
#endif

#define PI 3.14159265358979323846
static double wrap(double a) { return a - 2 * PI * floor((a + PI) / (2 * PI)); }

/* normal(0,1) (Box-Muller), semente fixa: ensaios simulados repetíveis */
static unsigned long long s_rng = 88172645463325252ull;
static double unif(void)
{
    s_rng ^= s_rng << 13, s_rng ^= s_rng >> 7, s_rng ^= s_rng << 17;
    return ((s_rng >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}
static double gauss(void) { return sqrt(-2 * log(unif())) * cos(2 * PI * unif()); }

int main(int argc, char **argv)
{
#ifdef _WIN32
    SetConsoleOutputCP(65001);   /* acentos no console */
#endif
    if (argc < 2) {
        fprintf(stderr, "uso: %s <config.ini> [saida.csv] [secao.chave=valor ...]\n", argv[0]);
        return 2;
    }
    config cfg;
    const char *livre;
    if (config_le_args(argc, argv, &cfg, &livre)) return 1;
    config_imprime(&cfg);
    const ensaio *e = &cfg.e;
    modelo m = modelo_de_fisico(cfg.f, e->Ts, cfg.mpc.dtheta_max);
    modelo planta = m;                            /* a planta pode ter atrito seco; o MPC não */
    planta.tc2 = e->tc2;
    if (e->mu2_planta > 0) planta.mu2 = e->mu2_planta;
    if (e->tc2 > 0) printf("planta com atrito seco tc2 = %g N·m (o MPC não modela)\n", e->tc2);
    double x[4] = {e->x0[0], e->x0[1], e->x0[2], e->x0[3]};
    if (e->semente > 0) {                         /* ensaio diferente a cada semente */
        s_rng += 0x9E3779B97F4A7C15ull * (unsigned long long)e->semente;
        x[1] += 0.1 * (unif() - 0.5);
    }
    estimador est;
    estimador_cria(&est, e->est_L1, e->est_L2);
    estimador_topo(&est, e->est_L1_topo, e->est_L2_topo, e->est_janela);
    double a_cmd = 0;                             /* último comando enviado */
    double xm[4] = {x[0], x[1], x[2], x[3]};      /* o que o controle vê */
    double th_est = x[1], w_est = x[3], fila[256] = {0};   /* fila: comando por ms (atraso) */
    long tms = 0;   /* observador de θ2 (como no firmware, 1 kHz) */
    double q = e->bits_enc > 0 ? 2 * PI / (1 << e->bits_enc) : 0;
    double ppr = e->passos_rev > 0 ? e->passos_rev / (2 * PI) : 0, th1_med = x[0], th1c = x[0], v_fw = x[2];
    double ws = 2 * PI * e->f_motor;              /* ressonância do rotor (0 = braço rígido) */
    if (e->planta_acel) {
        printf("planta comandada em aceleração | observador %g Hz | atraso %g ms | encoder %d bits (ruído %g LSB)\n",
               e->obs_hz, e->atraso_ms, e->bits_enc, e->ruido_enc);
        printf("motor: %g passos/volta | a ≤ %g, v ≤ %g, limite de software ±%g rad (0 = sem) | ressonância %g Hz, ζ %g\n",
               e->passos_rev, e->a_max_fw, e->v_max_fw, e->limite_fw, e->f_motor, e->zeta_motor);
    }
    mpc_config cfg_eq = config_mpc_equilibrio(&cfg);
    int troca = e->troca_equilibrio > 0 && (cfg.u_max_eq > 0 || cfg.Rdu_eq > 0), em_eq = 0, n_trocas = 0;
    double u_ant = 0;
    long volta = 0;                               /* dobramento de θ2 (wrap_theta2) */
    mpc *c = mpc_cria(&cfg.mpc, &m, x, 0.0);
    FILE *out = livre ? fopen(livre, "w") : NULL;
    if (out) fprintf(out, "t,theta1,theta2,dtheta1,dtheta2,theta1_ref,u,a,n_qp,n_it,dP,t_calc_ms,eq,theta1_real,theta2_real,dtheta2_real\n");

    int K = (int)lround(e->duracao / e->Ts);
    double t_up = -1, soma_tc = 0, max_tc = 0, soma_qp = 0, max_a = 0;
    int k;
    for (k = 0; k < K; k++) {
        double t = k * e->Ts;
        double ref[4] = {t >= e->t_degrau ? e->theta1_degrau : e->theta1_ref, 0, 0, 0};
        mpc_info in;
        double t0 = periodo_agora();
        if (e->estimador) estimador_passo(&est, &m, xm, a_cmd, e->Ts);
        double xc[4] = {xm[0], xm[1], xm[2], xm[3]};   /* compensação do atraso */
        if (e->comp_atraso_ms > 0) modelo_passo_acel(&m, xc, a_cmd, e->comp_atraso_ms * 1e-3, 4);
        if (e->wrap_theta2) {                         /* dobra θ2 com histerese: volta k tal que θ2 - 2πk fique perto de 0 */
            while (xc[1] - 2 * PI * volta > PI + e->wrap_hist) volta++;
            while (xc[1] - 2 * PI * volta < -PI - e->wrap_hist) volta--;
            xc[1] -= 2 * PI * volta;
        }
        if (troca) {                                  /* swing-up <-> equilíbrio, com histerese */
            int quer = fabs(xc[1]) < (em_eq ? 2 : 1) * e->troca_equilibrio;
            if (quer != em_eq) {
                mpc_libera(c);
                c = mpc_cria(quer ? &cfg_eq : &cfg.mpc, &m, xc, u_ant);
                em_eq = quer;
                n_trocas++;
            }
        }
        double u = mpc_passo(c, xc, ref, &in);
        u_ant = u;
        double tc = (periodo_agora() - t0) * 1e3;
        double a = modelo_acel_braco(&m, xc, u);
        a_cmd = a;
        soma_tc += tc;
        max_tc = fmax(max_tc, tc);
        soma_qp += in.n_qp;
        max_a = fmax(max_a, fabs(a));
        if (out)
            fprintf(out, "%.4f,%.10g,%.10g,%.10g,%.10g,%.6g,%.12g,%.8g,%d,%d,%.3e,%.4f,%d,%.10g,%.10g,%.10g\n", t, xm[0],
                    xm[1], xm[2], xm[3], ref[0], u, a, in.n_qp, in.n_it, in.dP, tc, em_eq, x[0], x[1], x[3]);
        if (t_up < 0 && fabs(wrap(x[1])) < 0.1 && fabs(x[3]) < 0.5) t_up = t;
        if (!in.finito) {
            printf("controle não finito em t = %.2f s\n", t);
            break;
        }
        double ud = u + (t >= e->t_disturbio && t < e->t_disturbio + e->dur_disturbio ? e->u_disturbio : 0);
        if (!e->planta_acel) {
            modelo_passo(&planta, x, ud, e->rk4_sub);
            for (int i = 0; i < 4; i++) xm[i] = x[i];
            continue;
        }
        /* 1 ms por vez: o comando novo entra após atraso_ms; o firmware lê o encoder e atualiza o observador */
        double ad = a + (ud - u) * 200;           /* distúrbio de torque ~ em aceleração (aprox.) */
        int nms = (int)lround(e->Ts * 1e3), nat = (int)lround(e->atraso_ms);
        if (nat > 200) nat = 200;
        for (int j = 0; j < nms; j++) fila[(tms + nat + j) & 255] = ad;
        for (int j = 0; j < nms; j++) {
            /* malha de 1 kHz do firmware (malha_rodando): satura a, integra v, freia antes do limite, satura v */
            double ac = fila[tms++ & 255], v0 = v_fw;
            if (e->a_max_fw > 0) ac = fmax(-e->a_max_fw, fmin(e->a_max_fw, ac));
            double v = v0 + ac * 1e-3;
            if (e->limite_fw > 0 && e->a_max_fw > 0) {
                double parada = v * v / (2 * e->a_max_fw);
                if (v > 0 && th1_med + parada >= e->limite_fw) v = fmax(v0 - e->a_max_fw * 1e-3, 0);
                if (v < 0 && th1_med - parada <= -e->limite_fw) v = fmin(v0 + e->a_max_fw * 1e-3, 0);
            }
            if (e->v_max_fw > 0) v = fmax(-e->v_max_fw, fmin(e->v_max_fw, v));
            v_fw = v;
            if (ws <= 0) {                              /* braço rígido: segue a velocidade comandada */
                modelo_passo_acel(&planta, x, (v - v0) * 1e3, 1e-3, 1);
                x[2] = v;
                th1c = x[0];
            } else {                                    /* rotor = mola até a posição dos passos (ressonância) */
                for (int i = 0; i < 10; i++) {
                    th1c += v * 1e-4;
                    double alvo = ppr > 0 ? floor(th1c * ppr) / ppr : th1c;
                    /* erro do micropasso: periódico a cada passo inteiro (200/volta) */
                    alvo += e->ripple_passo * (2 * PI / 200) * sin(200 * alvo);
                    double ar = ws * ws * (alvo - x[0]) + 2 * e->zeta_motor * ws * (v - x[2]);
                    modelo_passo_acel(&planta, x, ar, 1e-4, 1);
                }
            }
            /* θ1 medido = passos inteiros (acumulador do gerador de passos trunca); dθ1 = velocidade comandada */
            th1_med = ppr > 0 ? floor(th1c * ppr) / ppr : th1c;
            double ang = x[1] + (e->ruido_enc > 0 ? e->ruido_enc * q * gauss() : 0);
            double med = q > 0 ? q * floor(ang / q) : ang;
            if (e->obs_hz > 0) {
                double wn = 2 * PI * e->obs_hz, er = med - th_est;
                th_est += 1e-3 * (w_est + 2 * wn * er);
                w_est += 1e-3 * wn * wn * er;
            } else {
                w_est = x[3];
            }
            xm[0] = th1_med, xm[1] = med, xm[2] = v_fw, xm[3] = w_est;
        }
    }
    printf("passos %d | swing-up (|θ2| < 0,1 rad) em %.2f s | x final = (%.3f, %.3f, %.3f, %.3f)\n", k, t_up, x[0],
           x[1], x[2], x[3]);
    printf("t_calc: médio %.2f ms, máx %.2f ms | QPs por passo: %.2f | máx |θ̈1| = %.1f rad/s²\n", soma_tc / (k ? k : 1), max_tc,
           soma_qp / (k ? k : 1), max_a);
    if (troca) printf("trocas swing-up/equilíbrio: %d\n", n_trocas);
    if (out) fclose(out);
    mpc_libera(c);
    return 0;
}
