#include "qlpv_mpc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct mpc {
    mpc_config cfg;
    modelo m;
    int N, primeiro;
    double u_prev;
    chronos *ch;
    double *U, *S;          /* predição: U (N), S (4×N) */
    double *P, *Pn;         /* trajetória dos ρ: 8×N (coluna j = estágio j) */
    double *A, *B, *ref;    /* 4×4×N, 4×N, 4×N */
    double Pterm[16];
};

mpc_config mpc_config_artigo(void)
{
    mpc_config c = {0};
    c.Np = 40;
    c.Q[0] = 10, c.Q[1] = 100, c.Q[2] = 5, c.Q[3] = 1;
    c.Rdu = 100;
    for (int i = 0; i < 4; i++) c.peso_exp_estados[i] = 1;
    c.u_min = -0.6, c.u_max = 0.6;
    c.du_min = -10, c.du_max = 10;
    c.theta1_min = -3.1416, c.theta1_max = 3.1416;
    c.dtheta_max = 30;
    c.qv_theta1 = 1000, c.qv_dtheta = 1000, c.BIG = 1e3;
    c.rho_avg = 1;
    c.terminal_dlqr = 1;
    c.Ru_lqr = 1;
    c.l_max = 5;
    c.eps_rho = 1e-3;
    c.op = chronos_opcoes_padrao();
    return c;
}

/* A(P), B(P) por estágio (ZOH, com ρ médio entre o estágio j e o j+1 se rho_avg). */
static void dinamica(mpc *c, const double *P)
{
    for (int j = 0; j < c->N; j++) {
        double r[8];
        int jn = j + 1 < c->N ? j + 1 : j;
        for (int i = 0; i < 8; i++)
            r[i] = c->cfg.rho_avg ? 0.5 * (P[j * 8 + i] + P[jn * 8 + i]) : P[j * 8 + i];
        modelo_zoh(&c->m, r, c->A + 16 * j, c->B + 4 * j);
    }
}

/* s = rollout de U a partir de x com a dinâmica atual. */
static void rollout(const mpc *c, const double x[4], const double *U, double *s)
{
    double xk[4] = {x[0], x[1], x[2], x[3]}, t[4];
    for (int k = 0; k < c->N; k++) {
        const double *A = c->A + 16 * k, *B = c->B + 4 * k;
        for (int i = 0; i < 4; i++) {
            double acc = B[i] * U[k];
            for (int j = 0; j < 4; j++) acc += A[j * 4 + i] * xk[j];
            t[i] = acc;
        }
        memcpy(xk, t, sizeof(xk));
        memcpy(s + 4 * k, xk, sizeof(xk));
    }
}

/* P = (f_ρ(x), f_ρ(s_1), ..., f_ρ(s_{N-1})). */
static void rho_trajetoria(const mpc *c, const double x[4], const double *s, double *P)
{
    modelo_rho(&c->m, x, P);
    for (int j = 1; j < c->N; j++) modelo_rho(&c->m, s + 4 * (j - 1), P + 8 * j);
}

/* DARE por iteração de Riccati (nu = 1): P = Q + A'PA - A'PB (R + B'PB)^-1 B'PA. */
static void dlqr(const double A[16], const double B[4], const double Q[4], double R, double P[16])
{
    memset(P, 0, 16 * sizeof(double));
    for (int i = 0; i < 4; i++) P[i * 5] = Q[i];
    for (int it = 0; it < 200000; it++) {
        double AP[16], PB[4], BPB = R, APB[4], Pn[16];
        for (int i = 0; i < 4; i++) {                 /* PB = P·B, APB = A'·P·B */
            PB[i] = 0;
            for (int k = 0; k < 4; k++) PB[i] += P[k * 4 + i] * B[k];
        }
        for (int i = 0; i < 4; i++) BPB += B[i] * PB[i];
        for (int i = 0; i < 4; i++) {
            APB[i] = 0;
            for (int k = 0; k < 4; k++) APB[i] += A[i * 4 + k] * PB[k];     /* (A')_{ik} = A_{ki} */
        }
        for (int i = 0; i < 4; i++)                   /* AP = A'·P */
            for (int j = 0; j < 4; j++) {
                double acc = 0;
                for (int k = 0; k < 4; k++) acc += A[i * 4 + k] * P[j * 4 + k];
                AP[j * 4 + i] = acc;
            }
        double dif = 0;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                double acc = 0;                       /* (A'PA)_{ij} */
                for (int k = 0; k < 4; k++) acc += AP[k * 4 + i] * A[j * 4 + k];
                double v = (i == j ? Q[i] : 0) + acc - APB[i] * APB[j] / BPB;
                dif = fmax(dif, fabs(v - P[j * 4 + i]));
                Pn[j * 4 + i] = v;
            }
        for (int i = 0; i < 4; i++)                   /* simetriza a cada passo (a assimetria numérica diverge) */
            for (int j = 0; j < 4; j++) P[j * 4 + i] = 0.5 * (Pn[j * 4 + i] + Pn[i * 4 + j]);
        if (dif < 1e-12 * (1 + fabs(P[0]))) break;
    }
}

mpc *mpc_cria(const mpc_config *cfg, const modelo *m, const double x0[4], double u_prev)
{
    mpc *c = (mpc *)calloc(1, sizeof(mpc));
    int N = cfg->Np;
    c->cfg = *cfg;
    c->m = *m;
    c->N = N;
    c->U = (double *)calloc(N, sizeof(double));
    c->S = (double *)calloc(4 * N, sizeof(double));
    c->P = (double *)calloc(8 * N, sizeof(double));
    c->Pn = (double *)calloc(8 * N, sizeof(double));
    c->A = (double *)calloc(16 * N, sizeof(double));
    c->B = (double *)calloc(4 * N, sizeof(double));
    c->ref = (double *)calloc(4 * N, sizeof(double));

    double Qe[16] = {0};
    for (int i = 0; i < 4; i++) Qe[i * 5] = cfg->Q[i];
    double vmax = cfg->dtheta_max;
    double smin[4] = {cfg->theta1_min, -cfg->BIG, -vmax, -vmax};
    double smax[4] = {cfg->theta1_max, cfg->BIG, vmax, vmax};
    double qv[4] = {cfg->qv_theta1, cfg->qv_theta1, cfg->qv_dtheta, cfg->qv_dtheta};
    if (cfg->terminal_dlqr) {
        double r0[8], x_topo[4] = {0, 0, 0, 0}, A0[16], B0[4];
        modelo_rho(m, x_topo, r0);
        modelo_zoh(m, r0, A0, B0);
        dlqr(A0, B0, cfg->Q, cfg->Ru_lqr, c->Pterm);
    }
    chronos_restricoes rs = {1, 1, 1};
    c->ch = chronos_cria(4, 1, N, cfg->op, rs, Qe, &cfg->Rdu, cfg->terminal_dlqr ? c->Pterm : NULL, smin, smax, qv,
                         qv, &cfg->u_min, &cfg->u_max, &cfg->du_min, &cfg->du_max);

    if (cfg->peso_exp != 0) {                     /* longe da referência no começo, perto no fim */
        double *w = (double *)malloc(4 * N * sizeof(double));
        for (int k = 0; k < N; k++)
            for (int i = 0; i < 4; i++)
                w[4 * k + i] = cfg->peso_exp_estados[i] ? exp(cfg->peso_exp * ((k + 1.0) / N - 1.0)) : 1.0;
        chronos_peso_estagio(c->ch, w);
        free(w);
    }

    /* iterado inicial (build_chronos_mpc): u = u_prev saturado, modelo congelado em x0 */
    double eps = cfg->op.slack_epsilon;
    double u0 = fmin(fmax(u_prev, cfg->u_min + eps), cfg->u_max - eps);
    for (int k = 0; k < N; k++) c->U[k] = u0;
    for (int j = 0; j < N; j++) modelo_rho(m, x0, c->P + 8 * j);
    dinamica(c, c->P);
    chronos_dinamica(c->ch, c->A, c->B);
    chronos_inicializa(c->ch, x0, &u_prev);
    rollout(c, x0, c->U, c->S);
    c->u_prev = u_prev;
    c->primeiro = 1;
    return c;
}

void mpc_limite_du(mpc *c, double du_max)
{
    c->cfg.du_min = -du_max, c->cfg.du_max = du_max;
    chronos_limites_du(c->ch, &c->cfg.du_min, &c->cfg.du_max);
    /* projeta o U do warm start nos limites novos (o ponto interior precisa partir viável) */
    double e = c->cfg.op.slack_epsilon, up = c->u_prev;
    for (int k = 0; k < c->N; k++) {
        double v = fmin(fmax(c->U[k], up - du_max + e), up + du_max - e);
        v = fmin(fmax(v, c->cfg.u_min + e), c->cfg.u_max - e);
        c->U[k] = up = v;
    }
}

void mpc_libera(mpc *c)
{
    if (!c) return;
    chronos_libera(c->ch);
    free(c->U), free(c->S), free(c->P), free(c->Pn), free(c->A), free(c->B), free(c->ref);
    free(c);
}

double mpc_passo(mpc *c, const double x[4], const double ref[4], mpc_info *info)
{
    int N = c->N;
    if (!c->primeiro) {                                /* warm start: predição anterior deslocada */
        memmove(c->S, c->S + 4, 4 * (N - 1) * sizeof(double));
        memmove(c->U, c->U + 1, (N - 1) * sizeof(double));
        c->U[N - 1] = c->U[N - 2];
    }
    if (c->primeiro)
        for (int j = 0; j < N; j++) modelo_rho(&c->m, x, c->P + 8 * j);
    else
        rho_trajetoria(c, x, c->S, c->P);
    for (int k = 0; k < N; k++) memcpy(c->ref + 4 * k, ref, 4 * sizeof(double));

    mpc_info in = {0, 0, INFINITY, 0, 1};
    double *cu = chronos_u(c->ch), *csu = chronos_su(c->ch), *cs = chronos_s(c->ch);
    for (;;) {
        dinamica(c, c->P);
        chronos_dinamica(c->ch, c->A, c->B);
        memcpy(cu, c->U, N * sizeof(double));          /* partida: U atual, su = u, s = rollout */
        memcpy(csu, c->U, N * sizeof(double));
        rollout(c, x, c->U, cs);
        chronos_reinicia_folgas(c->ch, x, &c->u_prev);
        double u0;
        in.n_it += chronos_resolve(c->ch, x, &c->u_prev, c->ref, &u0);
        in.n_qp++;
        memcpy(c->U, cu, N * sizeof(double));
        memcpy(c->S, cs, 4 * N * sizeof(double));
        in.finito = 1;
        for (int i = 0; i < 4 * N && in.finito; i++) in.finito = isfinite(c->S[i]);
        for (int k = 0; k < N && in.finito; k++) in.finito = isfinite(c->U[k]);
        if (!in.finito) {
            in.dP = INFINITY;
            break;
        }
        rho_trajetoria(c, x, c->S, c->Pn);
        double dP = 0;
        for (int i = 0; i < 8 * N; i++) dP = fmax(dP, fabs(c->Pn[i] - c->P[i]));
        in.dP = dP;
        if (dP < c->cfg.eps_rho || in.n_qp >= c->cfg.l_max) break;
        double *t = c->P;                              /* Picard: P <- T(P) */
        c->P = c->Pn;
        c->Pn = t;
    }
    in.convergiu = in.dP < c->cfg.eps_rho;
    double u = in.finito ? fmin(fmax(c->U[0], c->cfg.u_min), c->cfg.u_max) : 0.0;
    c->u_prev = u;
    c->primeiro = 0;
    if (info) *info = in;
    return u;
}

const double *mpc_U(const mpc *c) { return c->U; }
const double *mpc_S(const mpc *c) { return c->S; }
const double *mpc_P_terminal(const mpc *c) { return c->Pterm; }
